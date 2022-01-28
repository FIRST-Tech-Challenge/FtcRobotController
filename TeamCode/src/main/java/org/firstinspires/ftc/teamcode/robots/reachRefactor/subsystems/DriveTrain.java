package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.TrikeDrive;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.TrikeKinematics;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.VoltageSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants.*;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import kotlin._Assertions;

@Config
public class DriveTrain extends TrikeDrive implements Subsystem {
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0.01, 0, 0.01);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.01, 0, 0);

    public static PIDCoefficients SWIVEL_PID_COEFFICIENTS = new PIDCoefficients(0.03, 0, 0.08);
    public static PIDCoefficients CHASSIS_LENGTH_PID_COEFFICIENTS = new PIDCoefficients(0.2, 0,  0.3);
    public static double SWIVEL_PID_TOLERANCE = 10;

    public static double VX_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static double MAX_ANGULAR_ACCELERATION = 1000;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectorySequenceRunner trajectorySequenceRunner;
    private TrajectoryFollower follower;

    private List<DcMotorEx> motors;
    private DcMotorEx leftMotor, rightMotor, swerveMotor, swivelMotor; // swerveMotor drives the module, swivelMotor rotates the module
    private DcMotorEx duckSpinner;
    private BNO055IMU imu;

    private DistanceSensor chassisLengthDistanceSensor;
    private VoltageSensor batteryVoltageSensor;

    private PIDController swivelPID, chassisLengthPID;

    // state
    private double leftPosition, rightPosition, swervePosition;
    private double swivelAngle, targetSwivelAngle;
    private double leftVelocity, rightVelocity, swerveVelocity, leftPower, rightPower, swervePower, duckSpinnerPower;
    private double chassisLength, targetChassisLength;
    private double heading, angularVelocity;
    private double angularAcceleration;
    private Pose2d driveVelocity, lastDriveVelocity;

    private long lastLoopTime, loopTime;

    private boolean maintainChassisLengthEnabled, antiTippingEnabled, smoothingEnabled;

    public DriveTrain(HardwareMap hardwareMap, boolean simulated) {
        super(kV, kA, kStatic, TRACK_WIDTH, simulated);
        follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID, new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        if(simulated) {
            chassisLengthDistanceSensor = new DistanceSensorSim(MIN_CHASSIS_LENGTH);
            batteryVoltageSensor = new VoltageSensorSim();

            leftMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            rightMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            swerveMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            swivelMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            duckSpinner = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            motors = Arrays.asList(leftMotor, rightMotor, swerveMotor, swivelMotor, duckSpinner);
        } else {
            chassisLengthDistanceSensor = hardwareMap.get(DistanceSensor.class, "distLength");
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            leftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
            rightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
            swerveMotor = hardwareMap.get(DcMotorEx.class, "motorMiddle");
            swivelMotor = hardwareMap.get(DcMotorEx.class, "motorMiddleSwivel");
            duckSpinner = hardwareMap.get(DcMotorEx.class,"duckSpinner");
            motors = Arrays.asList(leftMotor, rightMotor, swerveMotor, swivelMotor, duckSpinner);

            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);

                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            swivelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        swivelPID = new PIDController(SWIVEL_PID_COEFFICIENTS);
        chassisLengthPID = new PIDController(CHASSIS_LENGTH_PID_COEFFICIENTS);

        lastLoopTime = System.nanoTime();
    }

    @Override
    public void setDriveSignal(@NonNull DriveSignal driveSignal) {
        List<Double> velocities = TrikeKinematics.robotToWheelVelocities(driveSignal.getVel(), TRACK_WIDTH, getChassisLength());
        List<Double> accelerations = TrikeKinematics.robotToWheelAccelerations(driveSignal.getAccel(), TRACK_WIDTH, getChassisLength());
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);

        setMotorPowers(powers.get(0), powers.get(1), powers.get(2));
        setSwivelAngle(TrikeKinematics.robotToSwivelAngle(driveSignal.getVel(), getChassisLength()));

        driveVelocity = driveSignal.getVel();
    }

    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        List<Double> powers = TrikeKinematics.robotToWheelVelocities(drivePower, TRACK_WIDTH, getChassisLength());
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2));
    }


    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new TankVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    @Override
    public double getRawExternalHeading() {
        return heading;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return angularVelocity;
    }


    @Override
    public double getChassisLength() {
        return chassisLengthDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                leftPosition,
                rightPosition,
                swervePosition
        );
    }

    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                leftVelocity,
                rightVelocity,
                swerveVelocity
        );
    }

    public void setDuckSpinnerPower(double duckSpinnerPower) {
        this.duckSpinnerPower = duckSpinnerPower;
    }

    @Override
    public double getSwivelAngle() {
        return swivelAngle;
    }

    @Override
    public void setSwivelAngle(double targetSwivelAngle) {
        this.targetSwivelAngle = targetSwivelAngle;
    }

    @Override
    public void setMotorPowers(double leftPower, double rightPower, double swervePower) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.swervePower = swervePower;
    }

    private double getMaintainChassisLengthCorrection() {
        // returning 0 if distance sensor is likely blocked
        if(chassisLength < MIN_CHASSIS_LENGTH)
            return 0;

        // initialization of the PID calculator's output range, target value and multipliers
        chassisLengthPID.setOutputRange(-5.0, 5.0);
        chassisLengthPID.setPID(CHASSIS_LENGTH_PID_COEFFICIENTS);
        chassisLengthPID.setSetpoint(targetChassisLength);
        chassisLengthPID.enable();

        // initialization of the PID calculator's input range and current value
        chassisLengthPID.setInputRange(Constants.MIN_CHASSIS_LENGTH, Constants.MAX_CHASSIS_LENGTH);
        chassisLengthPID.setInput(chassisLength);

        // calculating correction
        return chassisLengthPID.performPID();
    }

    @Override
    public void update(Canvas fieldOverlay) {
        updatePoseEstimate();

        if(antiTippingEnabled) {
            angularAcceleration = (driveVelocity.getHeading() - lastDriveVelocity.getHeading()) / (loopTime * 1e-9);
            if(Math.abs(angularAcceleration) > MAX_ANGULAR_ACCELERATION) {
                double newAngularVelocity = lastDriveVelocity.getHeading() + Math.signum(angularAcceleration) * (loopTime * 1e-9);
                double newLinearVelocity = (driveVelocity.getHeading() / angularVelocity) * driveVelocity.getX();
                setDriveSignal(new DriveSignal(new Pose2d(new Vector2d(newLinearVelocity, 0), newAngularVelocity)));
            }
        }

        if(trajectorySequenceRunner.isBusy()) {
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity(), fieldOverlay);
            if (signal != null) setDriveSignal(signal);
        }

        leftPosition = encoderTicksToInches(leftMotor.getCurrentPosition());
        rightPosition = encoderTicksToInches(rightMotor.getCurrentPosition());
        swervePosition = encoderTicksToInches(swerveMotor.getCurrentPosition());

        leftVelocity = encoderTicksToInches(leftMotor.getVelocity());
        rightVelocity = encoderTicksToInches(rightMotor.getVelocity());
        swerveVelocity = encoderTicksToInches(swerveMotor.getVelocity());

        swivelAngle = UtilMethods.wrapAngleRad(swivelMotor.getCurrentPosition() / SWERVE_TICKS_PER_REVOLUTION * 2 * Math.PI);

        heading = imu.getAngularOrientation().firstAngle;
        angularVelocity = -imu.getAngularVelocity().xRotationRate;

        chassisLength = chassisLengthDistanceSensor.getDistance(DistanceUnit.INCH);

        if(maintainChassisLengthEnabled) {
            double chassisLengthCorrection = getMaintainChassisLengthCorrection();
            leftPower += chassisLengthCorrection;
            rightPower += chassisLengthCorrection;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        swerveMotor.setPower(swervePower);
        duckSpinner.setPower(duckSpinnerPower);

        long loopClockTime = System.nanoTime();
        loopTime = loopClockTime - lastLoopTime;
    }

    @Override
    public void stop() {
        leftPower = 0;
        rightPower = 0;
        swervePower = 0;
        swivelPID.disable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("x", getPoseEstimate().getX());
        telemetryMap.put("y", getPoseEstimate().getY());
        telemetryMap.put("heading", Math.toDegrees(getPoseEstimate().getHeading()));

        telemetryMap.put("xError", trajectorySequenceRunner.getLastPoseError().getX());
        telemetryMap.put("yError", trajectorySequenceRunner.getLastPoseError().getY());
        telemetryMap.put("headingError", Math.toDegrees(trajectorySequenceRunner.getLastPoseError().getHeading()));

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }

    // getters and setters

    public void setTargetLength(double targetChassisLength) {
        this.targetChassisLength = targetChassisLength;
    }

    public boolean isMaintainChassisLengthEnabled() {
        return maintainChassisLengthEnabled;
    }

    public void setMaintainChassisLengthEnabled(boolean maintainChassisLengthEnabled) {
        this.maintainChassisLengthEnabled = maintainChassisLengthEnabled;
    }

    public boolean isAntiTippingEnabled() {
        return antiTippingEnabled;
    }

    public void setAntiTippingEnabled(boolean antiTippingEnabled) {
        this.antiTippingEnabled = antiTippingEnabled;
    }

    public boolean isSmoothingEnabled() {
        return smoothingEnabled;
    }

    public void setSmoothingEnabled(boolean smoothingEnabled) {
        this.smoothingEnabled = smoothingEnabled;
    }

    public void setLeftPower(double leftPower) {
        this.leftPower = leftPower;
    }

    public void setRightPower(double rightPower) {
        this.rightPower = rightPower;
    }

    public void setSwervePower(double swervePower) {
        this.swervePower = swervePower;
    }
}
