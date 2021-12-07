package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.FFConstants.*;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.MathUtil.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.MathUtil;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.HashMap;
import java.util.Map;

public class DriveTrain implements Subsystem {

    // Motors
    private DcMotorEx motorFrontLeft, motorFrontRight, motorMiddle, motorMiddleSwivel, motorDuckSpinner;
    private DcMotorEx[] motors;
    private String[] MOTOR_NAMES = {"motorFrontLeft", "motorFrontRight", "motorMiddle", "motorMiddleSwivel", "motorDuckSpinner"};
    private boolean[] REVERSED = {true, false, false, false, false};
    private DcMotor.ZeroPowerBehavior[] ZERO_POWER_BEHAVIORS = new DcMotor.ZeroPowerBehavior[] {BRAKE, BRAKE, FLOAT, FLOAT, FLOAT};

    // Sensors
    BNO055IMU imu;
    private DistanceSensor sensorChassisDistance;

    // Kinematics
    private SimpleMatrix pose; // [x, y, yaw]
    private SimpleMatrix velocity; // [vx, vy, w]
    private SimpleMatrix angles; // [heading, roll, pitch]
    private SimpleMatrix offsetAngles; // [heading, roll, pitch]

    // PIVs
    private double targetFrontLeftVelocity, targetFrontRightVelocity, targetMiddleVelocity, targetSwivelAngle;
    private double swivelAngle;
    private double chassisDistance;
    private boolean imuInitialized;
    private boolean smoothingEnabled;

    // PID
    private PIDController turnPID, drivePID, swivelPID;

    // Smoothers
    private ExponentialSmoother frontLeftSmoother;
    private ExponentialSmoother frontRightSmoother;
    private ExponentialSmoother middleSmoother;

    // Constants
    public static final String TELEMETRY_NAME = "Drive Train";

    public DriveTrain(HardwareMap hardwareMap) {
        // Motors
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorMiddle= hardwareMap.get(DcMotorEx.class, "motorMiddle");
        motorMiddleSwivel = hardwareMap.get(DcMotorEx.class, "motorMiddleSwivel");
        motorDuckSpinner = hardwareMap.get(DcMotorEx.class, "motorDuckSpinner");
        motors = new DcMotorEx[] {motorFrontLeft, motorFrontRight, motorMiddle, motorMiddleSwivel, motorDuckSpinner};

        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(STOP_AND_RESET_ENCODER);
            motors[i].setMode(RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(ZERO_POWER_BEHAVIORS[i]);
            if(REVERSED[i])
                motors[i].setDirection(REVERSE);
        }

        // Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initializeIMU(hardwareMap);

        sensorChassisDistance = hardwareMap.get(DistanceSensor.class, "distLength");

        // PID
        turnPID = new PIDController(ROTATE_PID_COEFFICIENTS);
        drivePID = new PIDController(DRIVE_PID_COEFFICIENTS);
        swivelPID = new PIDController(SWIVEL_PID_COEFFICIENTS);

        // Smoother
        frontLeftSmoother = new ExponentialSmoother(FRONT_LEFT_SMOOTHING_FACTOR);
        frontRightSmoother = new ExponentialSmoother(FRONT_RIGHT_SMOOTHING_FACTOR);
        middleSmoother = new ExponentialSmoother(MIDDLE_SMOOTHING_FACTOR);
    }

    private void initializeIMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "baseIMU";

        imu.initialize(parametersIMU);
    }

    public void maintainSwerveAngle(double targetAngle) {
        //initialization of the PID calculator's output range, target value and multipliers
        swivelPID.setOutputRange(-1.0, 1.0);
        swivelPID.setPID(SWIVEL_PID_COEFFICIENTS);
        swivelPID.setSetpoint(targetAngle);
        swivelPID.enable();

        //initialization of the PID calculator's input range and current value
        swivelPID.setInputRange(0, 2 * Math.PI);
        swivelPID.setContinuous();
        swivelPID.setInput(swivelAngle);

        //calculates the angular correction to apply
        double correction = swivelPID.performPID();

        //performs the turn with the correction applied
        motorMiddleSwivel.setPower(correction);
    }

    private void updatePose() {
        Orientation imuAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        if (!imuInitialized) {
            // first time in - we assume that the robot has not started moving and that
            // orientation values are set to the current absolute orientation
            // so first set of imu readings are effectively offsets

            offsetAngles = new SimpleMatrix(new double[][] {
                    { (360 - imuAngles.firstAngle) % 360 },
                    { imuAngles.secondAngle % 360 },
                    { imuAngles.thirdAngle % 360 }
            });

            imuInitialized = true;
        }
        angles = new SimpleMatrix(new double[][] {
                {wrapAngle(360 - imuAngles.firstAngle, offsetAngles.get(0))},
                {wrapAngle(imuAngles.thirdAngle, offsetAngles.get(1))},
                {wrapAngle(imuAngles.secondAngle, offsetAngles.get(2))}
        });

        velocity = new SimpleMatrix(new double[][] {
                { imu.getVelocity().xVeloc },
                { imu.getVelocity().yVeloc },
                { imu.getAngularVelocity().xRotationRate }
        });

        pose = new SimpleMatrix(new double[][] {
                { imu.getPosition().x },
                { imu.getPosition().y },
                { angles.get(0) }
        });


    }

    public void drive(double linearVelocity, double angularVelocity) {
        double radius = angularVelocity == 0 ? 0 : linearVelocity / angularVelocity;

        targetFrontLeftVelocity = linearVelocity + angularVelocity * (radius - TRACK_WIDTH / 2);
        targetFrontRightVelocity = linearVelocity + angularVelocity * (radius + TRACK_WIDTH / 2);
        targetMiddleVelocity = linearVelocity + angularVelocity * Math.hypot(radius, chassisDistance);

        targetSwivelAngle = Math.atan2(chassisDistance, radius);

        if(smoothingEnabled) {
            targetFrontLeftVelocity = frontLeftSmoother.update(targetFrontLeftVelocity);
            targetFrontRightVelocity = frontRightSmoother.update(targetFrontLeftVelocity);
            targetMiddleVelocity = middleSmoother.update(targetMiddleVelocity);
        }
    }


    @Override
    public Map<String, Object> getTelemetry() {
        Map<String, Object> telemetryMap = new HashMap<String, Object>();
        telemetryMap.put("fl velocity", MathUtil.ticksToMeters(motorFrontLeft.getVelocity()));
        telemetryMap.put("fr velocity", MathUtil.ticksToMeters(motorFrontRight.getVelocity()));
        telemetryMap.put("middle velocity", MathUtil.ticksToMeters(motorMiddle.getVelocity()));

        telemetryMap.put("fl target velocity", targetFrontLeftVelocity);
        telemetryMap.put("fr target velocity", targetFrontRightVelocity);
        telemetryMap.put("middle target velocity", targetMiddleVelocity);

        telemetryMap.put("swivel angle", Math.toDegrees(swivelAngle));
        telemetryMap.put("target swivel angle", Math.toDegrees(targetSwivelAngle));

        return telemetryMap;
    }


    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void update() {
        motorFrontLeft.setVelocity(MathUtil.metersToTicks(targetFrontLeftVelocity));
        motorFrontRight.setVelocity(MathUtil.metersToTicks(targetFrontRightVelocity));
        motorMiddle.setVelocity(MathUtil.metersToTicks(targetFrontLeftVelocity));

        chassisDistance = sensorChassisDistance.getDistance(DistanceUnit.MM) * 1000;

        updatePose();

        swivelAngle = getSwivelAngle();
        maintainSwerveAngle(targetSwivelAngle);
    }

    @Override
    public void stop() {
        for(int i = 0; i < motors.length; i++) {
            motors[i].setPower(0);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

    private double getSwivelAngle() {
        return motorMiddleSwivel.getCurrentPosition() / DRIVETRAIN_TICKS_PER_REVOLUTION * 2 * Math.PI;
    }

    public double getChassisDistance() {
        return chassisDistance;
    }

    public SimpleMatrix getPose() {
        return pose;
    }

    public void toggleSmoothingEnabled() {
        smoothingEnabled = !smoothingEnabled;
    }
}
