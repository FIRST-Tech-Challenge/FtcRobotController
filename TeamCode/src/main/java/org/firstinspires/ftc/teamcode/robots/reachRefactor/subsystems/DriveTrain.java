package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.MathUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.HashMap;
import java.util.Map;

public class DriveTrain implements Subsystem {

    // Motors
    private DcMotorEx motorFrontLeft, motorFrontRight, motorMiddle, motorMiddleSwivel;
    private DcMotorEx[] motors;
    private String[] MOTOR_NAMES = {"motorFrontLeft", "motorFrontRight", "motorMiddle", "motorMiddleSwivel"};
    private boolean[] REVERSED = {true, false, false, false};
    private DcMotor.ZeroPowerBehavior[] ZERO_POWER_BEHAVIORS = new DcMotor.ZeroPowerBehavior[] {ZeroPowerBehavior.BRAKE, ZeroPowerBehavior.BRAKE, ZeroPowerBehavior.FLOAT, ZeroPowerBehavior.FLOAT};

    // Sensors
    private BNO055IMU imu;
    private DistanceSensor sensorChassisDistance;

    // Kinematics
    private SimpleMatrix pose; // [x, y, yaw]
    private SimpleMatrix velocity; // [vx, vy, w]
    private SimpleMatrix angles; // [heading, roll, pitch]
    private SimpleMatrix offsetAngles; // [heading, roll, pitch]
    private SimpleMatrix previousWheelTicks; // [left, right, middle]

    // PIVs
    private double targetFrontLeftVelocity, targetFrontRightVelocity, targetMiddleVelocity, targetSwivelAngle;
    private double targetLinearVelocity, targetAngularVelocity, targetTurnRadius;

    private double swivelAngle;
    private double chassisDistance, targetChassisDistance;
    private boolean smoothingEnabled;

    // PID
    private PIDController turnPID, drivePID, swivelPID, chassisDistancePID;

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
        motors = new DcMotorEx[] {motorFrontLeft, motorFrontRight, motorMiddle, motorMiddleSwivel};

        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(ZERO_POWER_BEHAVIORS[i]);
            if(REVERSED[i])
                motors[i].setDirection(Direction.REVERSE);
        }

        // Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initializeIMU();

        sensorChassisDistance = hardwareMap.get(DistanceSensor.class, "distLength");

        // Kinematics
        pose = new SimpleMatrix(3, 1);
        velocity = new SimpleMatrix(3, 1);
        angles = new SimpleMatrix(3, 1);
        offsetAngles = new SimpleMatrix(3, 1);
        previousWheelTicks = new SimpleMatrix(3, 2);

        // PID
        turnPID = new PIDController(Constants.ROTATE_PID_COEFFICIENTS);
        drivePID = new PIDController(Constants.DRIVE_PID_COEFFICIENTS);
        swivelPID = new PIDController(Constants.SWIVEL_PID_COEFFICIENTS);
        chassisDistancePID = new PIDController(Constants.CHASSIS_DISTANCE_PID_COEFFICIENTS);

        // Smoother
        frontLeftSmoother = new ExponentialSmoother(Constants.FRONT_LEFT_SMOOTHING_FACTOR);
        frontRightSmoother = new ExponentialSmoother(Constants.FRONT_RIGHT_SMOOTHING_FACTOR);
        middleSmoother = new ExponentialSmoother(Constants.MIDDLE_SMOOTHING_FACTOR);

        // Miscellaneous
        previousWheelTicks = getWheelTicks();
    }

    private void initializeIMU() {
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "baseIMU";

        imu.initialize(parametersIMU);

        // storing first absolute orientation values as offsets
        Orientation imuAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        offsetAngles = new SimpleMatrix(new double[][] {
                { (360 - imuAngles.firstAngle) % 360 },
                { imuAngles.secondAngle % 360 },
                { imuAngles.thirdAngle % 360 }
        });
    }

    private double getMaintainSwivelAngleCorrection() {
        //initialization of the PID calculator's output range, target value and multipliers
        swivelPID.setOutputRange(-1.0, 1.0);
        swivelPID.setPID(Constants.SWIVEL_PID_COEFFICIENTS);
        swivelPID.setSetpoint(targetSwivelAngle);
        swivelPID.enable();

        //initialization of the PID calculator's input range and current value
        swivelPID.setInputRange(0, 2 * Math.PI);
        swivelPID.setContinuous();
        swivelPID.setInput(swivelAngle);

        //calculates the angular correction to apply
        return swivelPID.performPID();
    }

    /**
     * updates the target chassis distance to maintain rotation without slipping
     */
    private void updateTargetChassisDistance() {
        double turnRadius = targetAngularVelocity == 0 ? 0 : targetLinearVelocity / targetAngularVelocity;
        targetChassisDistance = targetAngularVelocity == 0 ?
            // if not rotating, set target chassis distance to max length - threshold
            Constants.MAX_CHASSIS_LENGTH - Constants.CHASSIS_LENGTH_THRESHOLD :
            // when rotating, if calculated distance is smaller than minimum length, set to minimum length
            Math.max(
                Constants.MIN_CHASSIS_LENGTH + Constants.CHASSIS_LENGTH_THRESHOLD,
                // when rotating, if calculated distance is greater than maximum length, set to maximum length
                Math.min(
                    Constants.MAX_CHASSIS_LENGTH - Constants.CHASSIS_LENGTH_THRESHOLD,
                    Math.sqrt(
                        Math.pow(Constants.DRIVETRAIN_COEFFICIENT_OF_FRICTION * Constants.ACCELERATION_OF_GRAVITY / Math.pow(targetAngularVelocity, 2), 2)
                      - Math.pow(turnRadius, 2)
                )
            )
        );
    }

    private double getMaintainChassisDistanceCorrection() {
        // initialization of the PID calculator's output range, target value and multipliers
        swivelPID.setOutputRange(-1.0, 1.0);
        swivelPID.setPID(Constants.CHASSIS_DISTANCE_PID_COEFFICIENTS);
        swivelPID.setSetpoint(targetChassisDistance);
        swivelPID.enable();

        // initialization of the PID calculator's input range and current value
        swivelPID.setInputRange(0, Constants.MAX_CHASSIS_LENGTH);
        swivelPID.setInput(chassisDistance);

        // calculating correction
        return chassisDistancePID.performPID();
    }

    /**
     * updates the robot's pose ((x,y) position and heading) using the encoder ticks travelled by each wheel motor
     */
    private void updatePose() {
        Orientation imuAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angles = new SimpleMatrix(new double[][] {
                {MathUtils.wrapAngle(360 - imuAngles.firstAngle, offsetAngles.get(0))},
                {MathUtils.wrapAngle(imuAngles.thirdAngle, offsetAngles.get(1))},
                {MathUtils.wrapAngle(imuAngles.secondAngle, offsetAngles.get(2))}
        });

        // calculating wheel displacements
        SimpleMatrix wheelTicks = getWheelTicks().rows(0, 2);
        SimpleMatrix wheelDisplacementMeters = wheelTicks.minus(previousWheelTicks.rows(0, 2)).divide(Constants.DRIVETRAIN_TICKS_PER_METER);

//        // rotating swivel wheel by swivel angle
//        double swivelAngle = getSwivelAngle();
//        SimpleMatrix swivelWheel = MathUtil.rotateVector(
//                new SimpleMatrix(
//                        new double[][] {{ wheelDisplacementMeters.get(2, 0), 0 }}
//                ),
//                swivelAngle
//        );
//        wheelDisplacementMeters.setRow(2, 0, swivelWheel.get(0), swivelWheel.get(1));

        // calculating average average wheel displacement
        SimpleMatrix ones = new SimpleMatrix(new double[][] {{1, 1}});
        SimpleMatrix averageDisplacementMeters = ones.mult(wheelDisplacementMeters).divide(2);

        // rotating displacement by heading
        double heading = angles.get(0);
        averageDisplacementMeters = MathUtils.rotateVector(averageDisplacementMeters, heading);

        // updating pose [x, y, heading]
        pose = pose.plus(new SimpleMatrix(new double[][] {{
            averageDisplacementMeters.get(0, 0),
            averageDisplacementMeters.get(1, 0),
            0
        }}).transpose());
        pose.set(2, 0, angles.get(0));

        previousWheelTicks = wheelTicks.copy();
    }

    @Override
    public void update() {
        // state
        chassisDistance = sensorChassisDistance.getDistance(DistanceUnit.MM) / 1000 + Constants.DISTANCE_SENSOR_TO_FRONT_AXLE - Constants.DISTANCE_TARGET_TO_BACK_WHEEL;
        swivelAngle = (motorMiddleSwivel.getCurrentPosition() / (Constants.DRIVETRAIN_TICKS_PER_REVOLUTION * Constants.SWERVE_GEAR_RATIO) * 2 * Math.PI + Math.PI / 2) % (2 * Math.PI);;

        // PID corrections
        double maintainSwivelAngleCorrection = getMaintainSwivelAngleCorrection();
        motorMiddleSwivel.setPower(maintainSwivelAngleCorrection);

        updateTargetChassisDistance();
        double maintainChassisDistanceCorrection = getMaintainChassisDistanceCorrection();
        targetMiddleVelocity += maintainChassisDistanceCorrection;


        // Motor controls
        motorFrontLeft.setVelocity(targetFrontLeftVelocity * Constants.DRIVETRAIN_TICKS_PER_METER);
        motorFrontRight.setVelocity(targetFrontRightVelocity * Constants.DRIVETRAIN_TICKS_PER_METER);
        motorMiddle.setVelocity(targetFrontLeftVelocity * Constants.DRIVETRAIN_TICKS_PER_METER);

        updatePose();
    }

    /**
     * Drives the robot with the specified linear and angular velocities
     * @param linearVelocity the velocity, in m/s, to drive the robot
     * @param angularVelocity the angular velocity, in rad/s, to drive the robot
     */
    public void drive(double linearVelocity, double angularVelocity) {
        targetLinearVelocity = linearVelocity;
        targetAngularVelocity = angularVelocity;

        targetTurnRadius = angularVelocity == 0 ? 0 : linearVelocity / angularVelocity;

        targetFrontLeftVelocity = linearVelocity + angularVelocity * (targetTurnRadius - Constants.TRACK_WIDTH / 2);
        targetFrontRightVelocity = linearVelocity + angularVelocity * (targetTurnRadius + Constants.TRACK_WIDTH / 2);
        targetMiddleVelocity = linearVelocity + angularVelocity * Math.hypot(targetTurnRadius, chassisDistance);

        targetSwivelAngle = angularVelocity == 0 ? Math.PI / 2 : linearVelocity == 0 ? 0 : Math.atan2(chassisDistance, targetTurnRadius);

        if(smoothingEnabled) {
            targetFrontLeftVelocity = frontLeftSmoother.update(targetFrontLeftVelocity);
            targetFrontRightVelocity = frontRightSmoother.update(targetFrontLeftVelocity);
            targetMiddleVelocity = middleSmoother.update(targetMiddleVelocity);
        }
    }


    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        if(debug) {
            telemetryMap.put("fl velocity", MathUtils.ticksToMeters(motorFrontLeft.getVelocity()));
            telemetryMap.put("fr velocity", MathUtils.ticksToMeters(motorFrontRight.getVelocity()));
            telemetryMap.put("middle velocity", MathUtils.ticksToMeters(motorMiddle.getVelocity()));

            telemetryMap.put("fl target velocity", targetFrontLeftVelocity);
            telemetryMap.put("fr target velocity", targetFrontRightVelocity);
            telemetryMap.put("middle target velocity", targetMiddleVelocity);

            telemetryMap.put("swivel angle", Math.toDegrees(swivelAngle));
            telemetryMap.put("target swivel angle", Math.toDegrees(targetSwivelAngle));

            telemetryMap.put("chassis distance (m)", chassisDistance);

            telemetryMap.put("pose (x)", pose.get(0));
            telemetryMap.put("pose (y)", pose.get(1));
            telemetryMap.put("pose (heading)", Math.toDegrees(pose.get(2)));
        }

        return telemetryMap;
    }


    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void reset() {
        // reset IMU
        initializeIMU();

        // reset motors
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i].setMode(RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(RunMode.RUN_USING_ENCODER);
        }

        // reset smoothers
        frontLeftSmoother = new ExponentialSmoother(Constants.FRONT_LEFT_SMOOTHING_FACTOR);
        frontRightSmoother = new ExponentialSmoother(Constants.FRONT_RIGHT_SMOOTHING_FACTOR);
        middleSmoother = new ExponentialSmoother(Constants.MIDDLE_SMOOTHING_FACTOR);
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

    public double getSwivelAngle() {
        return swivelAngle;
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

    public boolean isSmoothingEnabled() {
        return smoothingEnabled;
    }

    /**
     * returns the current motor encoder positions for all three drivetrain motors:
     * [left, right, middle]
     * @return
     */
    public SimpleMatrix getWheelTicks() {
        return new SimpleMatrix(new double[][] {
                { motorFrontLeft.getCurrentPosition(), 0 },
                { motorFrontRight.getCurrentPosition(), 0 },
                { motorMiddle.getCurrentPosition(), 0 }
        });
    }

    public double getTurnRadius() {
        return targetTurnRadius;
    }
}
