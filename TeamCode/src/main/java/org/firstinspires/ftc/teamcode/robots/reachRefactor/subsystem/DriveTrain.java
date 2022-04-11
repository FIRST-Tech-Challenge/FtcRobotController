package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.CRServoSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.CloneFollower;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.TrikeKinematics;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants.*;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Mahesh Natamai
 */

@Config
public class DriveTrain extends TrikeDrive implements Subsystem {
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public static double B = 0.005;
    public static double ZETA = 0.01;
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(4, 0, 0);
    public static PIDCoefficients CROSS_AXIAL_PID = new PIDCoefficients(0.001, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(4.5, 0, 0);

    public static PIDCoefficients CUSTOM_HEADING_PID = new PIDCoefficients(1.5, 0.0, 1.0);
    public static double CUSTOM_HEADING_PID_TOLERANCE = 2;

    public static PIDCoefficients DIST_TRAVELLED_PID = new PIDCoefficients(5, 0.0, 0);

    public static PIDCoefficients ROLL_ANTI_TIP_PID = new PIDCoefficients(10, 0, 0);
    public static double ROLL_ANTI_TIP_PID_TOLERANCE = 2;
    public static PIDCoefficients PITCH_ANTI_TIP_PID = new PIDCoefficients(0, 0, 0);
    public static double PITCH_ANTI_TIP_PID_TOLERANCE = 5;

    public static PIDCoefficients SWIVEL_PID = new PIDCoefficients(1, 0, 0.08);
    public static PIDCoefficients CHASSIS_LENGTH_PID = new PIDCoefficients(3, 0, 2);
    public static PIDCoefficients AUTON_CHASSIS_LENGTH_PID = new PIDCoefficients(0.75, 0, 0);
    public static double CHASSIS_LENGTH_TOLERANCE = 1;
    public static PIDCoefficients MAINTAIN_HEADING_PID = new PIDCoefficients(1, 0, 0.5);
    public static double MAINTAIN_HEADING_TOLERANCE = 2.5;
    public static double SWIVEL_TOLERANCE = 1;
    public static double DUCK_SPINNER_POWER = 0.5;

    public final TrajectorySequenceRunner trajectorySequenceRunner;

    private final List<DcMotorEx> motors;
    private final DcMotorEx leftMotor, rightMotor, swerveMotor, swivelMotor; // swerveMotor drives the module,
                                                                             // swivelMotor rotates the module
    private final CRServo duckSpinner;
    private final BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;
    private final DistanceSensor chassisLengthDistanceSensor;

    private final PIDController swivelPID, chassisLengthPID, rollAntiTipPID, pitchAntiTipPID, maintainHeadingPID;
    private PIDController headingPID, distTravelledPID;

    private final boolean simulated;

    private double leftPosition, rightPosition, leftRelOffset, rightRelOffset, swervePosition, swivelPosition;
    private double swivelAngle, targetSwivelAngle;
    private double leftVelocity, rightVelocity, swerveVelocity;
    private double targetLeftVelocity, targetRightVelocity, targetSwerveVelocity, swivelPower, duckSpinnerPower;
    private double leftPower, rightPower, swervePower;
    private boolean useMotorPowers;
    private double chassisLength, targetChassisLength, chassisLengthCorrection;
    private boolean maintainChassisLengthEnabled, maintainHeadingEnabled, duckSpinnerToggled, imuOffsetsInitialized,
            duckGameEnabled, useAutonChassisLengthPID;
    private boolean chassisLengthOnTarget, maintainHeadingOnTarget;
    private double maintainHeading, maintainHeadingCorrection;
    private double heading, roll, pitch, pitchVelocity, angularVelocity;
    private double headingOffset, rollOffset, pitchOffset;
    private double rollCorrection, pitchCorrection;
    private double compensatedBatteryVoltage;

    private Pose2d poseEstimate, poseError, poseVelocity;
    private Pose2d driveVelocity, lastDriveVelocity;

    private long lastLoopTime, loopTime;

    private ChassisLengthMode chassisLengthMode;

    public enum ChassisLengthMode {
        SWERVE, DIFF, BOTH
    }

    public DriveTrain(HardwareMap hardwareMap, boolean simulated) {
        super(simulated);
        this.simulated = simulated;
        // TrajectoryFollower follower = new RamseteFollower(B, ZETA, new Pose2d(0.5,
        // 0.5, Math.toRadians(5)), 3);
        TrajectoryFollower follower = new CloneFollower(AXIAL_PID, CROSS_AXIAL_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5)), 1.5);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        if (simulated) {
            chassisLengthDistanceSensor = new DistanceSensorSim(
                    MIN_CHASSIS_LENGTH - (DISTANCE_SENSOR_TO_FRONT_AXLE + DISTANCE_TARGET_TO_BACK_WHEEL));

            leftMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            rightMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            swerveMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            swivelMotor = new DcMotorExSim(USE_MOTOR_SMOOTHING);
            duckSpinner = new CRServoSim();
            motors = Arrays.asList(leftMotor, rightMotor, swerveMotor, swivelMotor);

            compensatedBatteryVoltage = 14.0;
        } else {
            chassisLengthDistanceSensor = hardwareMap.get(DistanceSensor.class, "distLength");
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            leftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
            rightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
            swerveMotor = hardwareMap.get(DcMotorEx.class, "motorMiddle");
            swivelMotor = hardwareMap.get(DcMotorEx.class, "motorMiddleSwivel");
            duckSpinner = hardwareMap.get(CRServo.class, "duckSpinner");
            motors = Arrays.asList(leftMotor, rightMotor, swerveMotor, swivelMotor);

            for (DcMotorEx motor : motors) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);

                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                compensatedBatteryVoltage = batteryVoltageSensor.getVoltage();
                updateVelocityCoefficients();
            }

            swivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            swivelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        swivelPID = new PIDController(SWIVEL_PID);
        swivelPID.setInputRange(0, Math.toRadians(360));
        swivelPID.setContinuous(true);
        swivelPID.setTolerance(SWIVEL_TOLERANCE);
        swivelPID.enable();

        chassisLengthPID = new PIDController(CHASSIS_LENGTH_PID);
        chassisLengthPID.setInputRange(MIN_CHASSIS_LENGTH, MAX_CHASSIS_LENGTH);
        chassisLengthPID.setOutputRange(-100, 100);
        chassisLengthPID.setTolerance(CHASSIS_LENGTH_TOLERANCE);
        chassisLengthPID.enable();

        maintainHeadingPID = new PIDController(MAINTAIN_HEADING_PID);
        maintainHeadingPID.setInputRange(0, Math.toRadians(360));
        maintainHeadingPID.setOutputRange(-100, 100);
        maintainHeadingPID.setContinuous(true);
        maintainHeadingPID.setTolerance(MAINTAIN_HEADING_TOLERANCE);
        maintainHeadingPID.enable();

        rollAntiTipPID = new PIDController(ROLL_ANTI_TIP_PID);
        rollAntiTipPID.setInputRange(0, Math.toRadians(360));
        rollAntiTipPID.setOutputRange(-100, 100);
        rollAntiTipPID.setContinuous(true);
        rollAntiTipPID.setSetpoint(rollOffset);
        rollAntiTipPID.setTolerance(ROLL_ANTI_TIP_PID_TOLERANCE);
        rollAntiTipPID.enable();

        pitchAntiTipPID = new PIDController(PITCH_ANTI_TIP_PID);
        pitchAntiTipPID.setInputRange(0, Math.toRadians(360));
        pitchAntiTipPID.setOutputRange(-100, 100);
        pitchAntiTipPID.setContinuous(true);
        pitchAntiTipPID.setSetpoint(pitchOffset);
        pitchAntiTipPID.setTolerance(PITCH_ANTI_TIP_PID_TOLERANCE);
        pitchAntiTipPID.enable();

        headingPID = new PIDController(CUSTOM_HEADING_PID);
        headingPID.setInputRange(0, Math.toRadians(360));
        headingPID.setOutputRange(-100, 100);
        headingPID.setContinuous(true);
        headingPID.setTolerance(CUSTOM_HEADING_PID_TOLERANCE);
        headingPID.enable();

        //oof currently this will be in inches units
        //input is in inches, output is drive speed
        distTravelledPID = new PIDController(DIST_TRAVELLED_PID);
        distTravelledPID.setInputRange(-144, 144);
        distTravelledPID.setOutputRange(-30, 30); //max speed for Reach
        distTravelledPID.setContinuous(false);
        distTravelledPID.setTolerance(1);
        distTravelledPID.enable();

        driveVelocity = new Pose2d(0, 0, 0);
        lastDriveVelocity = new Pose2d(0, 0, 0);
        chassisLengthMode = ChassisLengthMode.BOTH;
    }

    private double getSwivelAngleCorrection() {
        swivelPID.setInput(swivelAngle);
        swivelPID.setSetpoint(targetSwivelAngle);

        return swivelPID.performPID();
    }

    private double getChassisLengthCorrection() {
        chassisLengthPID.setInput(chassisLength);
        chassisLengthPID.setSetpoint(targetChassisLength);

        return chassisLength < MIN_CHASSIS_LENGTH && targetChassisLength == MIN_CHASSIS_LENGTH ? 0 : chassisLengthPID.performPID();
    }

    private void updateVelocityCoefficients() {
        if (USE_CUSTOM_VELOCITY_PID) {
            PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                    DIFF_MOTOR_VELOCITY_PID.p, DIFF_MOTOR_VELOCITY_PID.i, DIFF_MOTOR_VELOCITY_PID.d,
                    DIFF_MOTOR_VELOCITY_PID.f * 12 / compensatedBatteryVoltage);
            for (DcMotorEx motor : motors) {
                motor.setVelocityPIDFCoefficients(
                        compensatedCoefficients.p,
                        compensatedCoefficients.i,
                        compensatedCoefficients.d,
                        compensatedCoefficients.f);
            }
            swerveMotor.setVelocityPIDFCoefficients(
                    SWERVE_VELOCITY_PID.p,
                    SWERVE_VELOCITY_PID.i,
                    SWERVE_VELOCITY_PID.d,
                    SWERVE_VELOCITY_PID.f);
        }
    }

    private void updatePIDCoefficients() {
        swivelPID.setPID(SWIVEL_PID);
        swivelPID.setTolerance(SWIVEL_TOLERANCE);

        chassisLengthPID.setPID(useAutonChassisLengthPID ? AUTON_CHASSIS_LENGTH_PID : CHASSIS_LENGTH_PID);
        chassisLengthPID.setTolerance(CHASSIS_LENGTH_TOLERANCE);

        maintainHeadingPID.setPID(MAINTAIN_HEADING_PID);
        maintainHeadingPID.setTolerance(MAINTAIN_HEADING_TOLERANCE);

        rollAntiTipPID.setPID(ROLL_ANTI_TIP_PID);
        rollAntiTipPID.setTolerance(ROLL_ANTI_TIP_PID_TOLERANCE);

        pitchAntiTipPID.setPID(PITCH_ANTI_TIP_PID);
        pitchAntiTipPID.setTolerance(PITCH_ANTI_TIP_PID_TOLERANCE);

        headingPID.setPID(CUSTOM_HEADING_PID);
        headingPID.setTolerance(CUSTOM_HEADING_PID_TOLERANCE);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        updateVelocityCoefficients();
        updatePIDCoefficients();

        // sensor readings
        leftVelocity = diffEncoderTicksToInches(leftMotor.getVelocity());
        rightVelocity = diffEncoderTicksToInches(rightMotor.getVelocity());
        swerveVelocity = swerveEncoderTicksToInches(swerveMotor.getVelocity());
        if (simulated) {
            double dt = loopTime / 1e9;
            leftPosition += leftVelocity * dt;
            rightPosition += rightVelocity * dt;
            swervePosition += swerveVelocity * dt;
            swivelPosition += swivelPower * dt;
            chassisLength = targetChassisLength;
        } else {
            leftPosition = diffEncoderTicksToInches(leftMotor.getCurrentPosition() - leftRelOffset);
            rightPosition = diffEncoderTicksToInches(rightMotor.getCurrentPosition() - rightRelOffset);
            swervePosition = swerveEncoderTicksToInches(swerveMotor.getCurrentPosition()); //todo fixup relative swerve position?
            swivelPosition = swivelMotor.getCurrentPosition();
            swivelAngle = wrapAngleRad(swivelPosition / SWIVEL_TICKS_PER_REVOLUTION * Math.toRadians(360));
            chassisLength = chassisLengthDistanceSensor.getDistance(DistanceUnit.INCH) + DISTANCE_SENSOR_TO_FRONT_AXLE
                    + DISTANCE_TARGET_TO_BACK_WHEEL;
        }

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        if (!imuOffsetsInitialized && imu.isGyroCalibrated()) {
            headingOffset = wrapAngleRad(orientation.firstAngle);
            rollOffset = wrapAngleRad(orientation.secondAngle);
            pitchOffset = wrapAngleRad(orientation.thirdAngle);

            imuOffsetsInitialized = true;
        }

        heading = wrapAngleRad(orientation.firstAngle - headingOffset);
        roll = wrapAngleRad(orientation.secondAngle - rollOffset);
        pitch = wrapAngleRad(orientation.thirdAngle - pitchOffset);

        AngularVelocity angularVelocities = imu.getAngularVelocity();
        pitchVelocity = wrapAngleRad(angularVelocities.yRotationRate);
        angularVelocity = wrapAngleRad(angularVelocities.xRotationRate);
        if (angularVelocity > Math.PI)
            angularVelocity -= 2 * Math.PI;

        updatePoseEstimate();
        poseEstimate = getPoseEstimate();
        poseVelocity = getPoseVelocity();

        if(trajectorySequenceRunner.isBusy()) {
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity(), fieldOverlay);
            if (signal != null)
                setDriveSignal(signal);
            poseError = trajectorySequenceRunner.getLastPoseError();
        }

        // PIDs
        rollAntiTipPID.setInput(roll);
        rollCorrection = rollAntiTipPID.performPID();
        pitchAntiTipPID.setInput(pitch);
        pitchCorrection = pitchAntiTipPID.performPID();

        if (rollAntiTipPID.onTarget())
            rollCorrection = 0;
        if (pitchAntiTipPID.onTarget())
            pitchCorrection = 0;

        Pose2d effectiveDriveVelocity = driveVelocity;
        if (maintainHeadingEnabled) {
            maintainHeadingPID.setInput(poseEstimate.getHeading());
            maintainHeadingPID.setSetpoint(maintainHeading);
            maintainHeadingCorrection = maintainHeadingPID.performPID();
            maintainHeadingOnTarget = maintainHeadingPID.onTarget();

            if (!maintainHeadingOnTarget)
                effectiveDriveVelocity = effectiveDriveVelocity.plus(new Pose2d(0, 0, maintainHeadingCorrection));
        }
        // sending corrections for chassis length
        chassisLengthOnTarget = chassisLengthPID.onTarget();
        if (maintainChassisLengthEnabled && !useMotorPowers) {
            chassisLengthCorrection = getChassisLengthCorrection();

            List<Double> frontVelocities = TrikeKinematics.robotToWheelVelocities(
                    effectiveDriveVelocity.plus(
                            new Pose2d(
                                    chassisLengthMode == ChassisLengthMode.DIFF
                                            || chassisLengthMode == ChassisLengthMode.BOTH ? chassisLengthCorrection
                                                    : 0,
                                    0, 0)),
                    TRACK_WIDTH, chassisLength);
            List<Double> backVelocities = TrikeKinematics.robotToWheelVelocities(
                    effectiveDriveVelocity.plus(
                            new Pose2d(
                                    chassisLengthMode == ChassisLengthMode.SWERVE
                                            || chassisLengthMode == ChassisLengthMode.BOTH ? -chassisLengthCorrection
                                                    : 0,
                                    0, 0)),
                    TRACK_WIDTH, chassisLength);
            setMotorVelocities(frontVelocities.get(0), frontVelocities.get(1), backVelocities.get(2));
            if (!approxEquals(frontVelocities.get(0), 0) ||
                    !approxEquals(frontVelocities.get(1), 0) ||
                    !approxEquals(backVelocities.get(2), 0))
                setSwivelAngle(TrikeKinematics.robotToSwivelAngle(
                        effectiveDriveVelocity.plus(
                                new Pose2d(
                                        chassisLengthMode == ChassisLengthMode.SWERVE
                                                || chassisLengthMode == ChassisLengthMode.BOTH
                                                        ? -chassisLengthCorrection
                                                        : 0,
                                        0, 0)),
                        chassisLength));
        }

        // swerve optimizations
        double setpointAngle = closestAngle(swivelAngle, targetSwivelAngle);
        double setpointAngleFlipped = closestAngle(swivelAngle, targetSwivelAngle + Math.toRadians(180));
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)) {
            // unflip the motor direction use the setpoint
            targetSwivelAngle = wrapAngleRad(swivelAngle + setpointAngle);
        }
        // if the closest angle to setpoint + 180 is shorter
        else {
            // flip the motor direction and use the setpoint + 180
            targetSwerveVelocity *= -1;
            targetSwivelAngle = wrapAngleRad(swivelAngle + setpointAngleFlipped);
        }
        if (simulated)
            swivelAngle = targetSwivelAngle;
        swivelPower = getSwivelAngleCorrection();

        if (useMotorPowers) {
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
            swerveMotor.setPower(swervePower);
        } else {
            leftMotor.setVelocity(diffInchesToEncoderTicks(targetLeftVelocity));
            rightMotor.setVelocity(diffInchesToEncoderTicks(targetRightVelocity));
            swerveMotor.setVelocity(swerveInchesToEncoderTicks(targetSwerveVelocity));
        }
        swivelMotor.setPower(swivelPower);
        duckSpinner.setPower(-duckSpinnerPower);

        lastDriveVelocity = driveVelocity;

        long loopClockTime = System.nanoTime();
        loopTime = loopClockTime - lastLoopTime;
        lastLoopTime = loopClockTime;
    }

    @Override
    public void stop() {
        setChassisLength(chassisLength);
        setSwivelAngle(swivelAngle);
        trajectorySequenceRunner.stop();
        setMotorVelocities(0, 0, 0);
        duckSpinnerPower = 0;
        driveVelocity = new Pose2d(0, 0, 0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("turnStuff", turnAngle - poseEstimate.getHeading());

        if (debug) {
            telemetryMap.put("x", poseEstimate.getX());
            telemetryMap.put("y", poseEstimate.getY());
            telemetryMap.put("heading", Math.toDegrees(poseEstimate.getHeading()));

            telemetryMap.put("x vel", poseVelocity.getX());
            telemetryMap.put("y vel", poseVelocity.getY());
            telemetryMap.put("heading vel", Math.toDegrees(poseVelocity.getHeading()));

            if (trajectorySequenceRunner.isBusy()) {
                telemetryMap.put("xError", poseError.getX());
                telemetryMap.put("yError", poseError.getY());
                telemetryMap.put("headingError", Math.toDegrees(poseError.getHeading()));
            }

            telemetryMap.put("roll", Math.toDegrees(roll));
            telemetryMap.put("pitch", Math.toDegrees(pitch));
            telemetryMap.put("anti-tip roll correction", Math.toDegrees(rollCorrection));
            telemetryMap.put("anti-tip pitch correction", pitchCorrection);
            telemetryMap.put("anti-tip roll on target", rollAntiTipPID.onTarget());
            telemetryMap.put("anti-tip pitch on target", pitchAntiTipPID.onTarget());

            telemetryMap.put("left position", leftPosition);
            telemetryMap.put("right position", rightPosition);
            telemetryMap.put("left position tics", diffInchesToEncoderTicks(leftPosition));
            telemetryMap.put("right position tics", diffInchesToEncoderTicks(rightPosition));
            telemetryMap.put("swerve position", swerveInchesToEncoderTicks(swervePosition));

            telemetryMap.put("swivel position", swivelPosition);
            telemetryMap.put("swivel angle", Math.toDegrees(swivelAngle));
            telemetryMap.put("target swivel angle", Math.toDegrees(targetSwivelAngle));

            telemetryMap.put("left velocity", leftVelocity);
            telemetryMap.put("right velocity", rightVelocity);
            telemetryMap.put("swerve velocity", swerveVelocity);

            telemetryMap.put("target left velocity", targetLeftVelocity);
            telemetryMap.put("target right velocity", targetRightVelocity);
            telemetryMap.put("target swerve velocity", targetSwerveVelocity);
            telemetryMap.put("swivel power", swivelPower);
            telemetryMap.put("duck spinner power", duckSpinnerPower);

            telemetryMap.put("chassis length", chassisLength);
            telemetryMap.put("target chassis length", targetChassisLength);
            telemetryMap.put("chassis length mode", chassisLengthMode);
            telemetryMap.put("chassis length PID on target", chassisLengthOnTarget);
            telemetryMap.put("chassis length PID correction", chassisLengthCorrection);

            telemetryMap.put("maintain heading enabled", maintainHeadingEnabled);
            telemetryMap.put("maintain heading", Math.toDegrees(maintainHeading));
            telemetryMap.put("maintain heading PID on target", maintainHeadingOnTarget);
            telemetryMap.put("maintain heading PID correction", maintainHeadingCorrection);

            telemetryMap.put("angular velocity", Math.toDegrees(angularVelocity));
            telemetryMap.put("pitch velocity", Math.toDegrees(pitchVelocity));

            telemetryMap.put("drive velocity", driveVelocity.toString());
            telemetryMap.put("last drive velocity", lastDriveVelocity.toString());

            telemetryMap.put("loop time", loopTime / 1e9);

            telemetryMap.put("maintain chassis length enabled", maintainChassisLengthEnabled);
            if (!simulated) {
                PIDFCoefficients velocityCoefficients = leftMotor
                        .getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetryMap.put("measured drivetrain PID coeffs", String.format("(p: %f, i: %f, d: %f)",
                        velocityCoefficients.p, velocityCoefficients.i, velocityCoefficients.d));
            }
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Drive Train";
    }

    // ----------------------------------------------------------------------------------------------
    // Trajectory Following
    // ----------------------------------------------------------------------------------------------

    private double getAveragePos() {
        return (leftPosition + rightPosition) / 2.0;
    }

    //reset the relative position to zeros
    private void resetRelPos(){
        leftRelOffset = leftMotor.getCurrentPosition();
        rightRelOffset = rightMotor.getCurrentPosition();
        leftPosition = 0;
        rightPosition = 0;
    }

    private double driveTarget, driveHeading, driveSpeed, driveDistErr, driveDistErrPrev;
    private Stage driveStage = new Stage();
    private StateMachine drive = Utils.getStateMachine(driveStage)
            .addState(() -> {
                headingPID.setSetpoint(driveHeading);
                headingPID.setInput(poseEstimate.getHeading());
                double correction = headingPID.performPID();
                //driveDistErr=Math.abs(driveTarget - getAveragePos());
                distTravelledPID.setSetpoint(driveTarget);
                distTravelledPID.setInput(getAveragePos());
                double spd = distTravelledPID.performPID();
                setDriveSignal(new DriveSignal(new Pose2d(spd, 0, correction), new Pose2d(0, 0, 0)));
                return (distTravelledPID.onTarget());
            })
            .build();

    boolean driveUntilInitialized = false;

    //call this version if we just want to continue in the direction the robot is currently pointing
    public boolean driveUntil(double driveDistance, double driveSpeed) {
        return(driveUntil(driveDistance, poseEstimate.getHeading(), driveSpeed));
    }

    public boolean driveUntil(double driveDistance, double driveHeading, double driveSpeed) {
        if(!driveUntilInitialized) {
            resetRelPos();
            this.driveTarget = driveDistance + getAveragePos();
            this.driveHeading = driveHeading;
            this.driveSpeed = driveSpeed;
            distTravelledPID.setOutputRange(-driveSpeed,driveSpeed); //max speed
            distTravelledPID.setSetpoint(driveTarget);
            driveUntilInitialized = true;
        }

        if(drive.execute()){
            setDriveSignal(new DriveSignal(new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)));

            driveUntilInitialized = false;
            return true;
        }
        return false;
    }
    //version using a heading requested in degrees
    public boolean driveUntilDegrees(double driveDistance, double driveHeading, double driveSpeed) {
        return driveUntil(driveDistance, Math.toRadians(driveHeading), driveSpeed);
    }

    //driveAsyncInitialized is only true when its currently driving
    boolean isDriving(){return driveUntilInitialized;}

    private double turnError = 2.0;
    private double turnAngle;
    private Stage turnStage = new Stage();
    private StateMachine turn = Utils.getStateMachine(turnStage)
            .addState(() -> {
                headingPID.setSetpoint(turnAngle);
                headingPID.setInput(poseEstimate.getHeading());
                double correction = headingPID.performPID();
                setDriveSignal(new DriveSignal(new Pose2d(0, 0, correction), new Pose2d(0, 0, 0)));
                return headingPID.onTarget();
            })
            .build();

    boolean turnInit = false;
    public boolean turnUntil(double turnAngle) {
        if(!turnInit){
            this.turnAngle = turnAngle; // this is in Radians relative to the starting angle as set by auton alliance
            turnInit = true;
        }

        if(turn.execute()){
            setDriveSignal(new DriveSignal(new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)));
            turnInit = false;
            return true;
        }
        return false;
    }
    //request a turn in degrees units
    public boolean turnUntilDegrees(double turnAngle) {
        return turnUntil(Math.toRadians(turnAngle));
    }

    //see isDriving();
    boolean isTurning(){return turnInit;}

    @Override
    public void setDriveSignal(@NonNull DriveSignal driveSignal) {
        useMotorPowers = false;
        List<Double> velocities = TrikeKinematics.robotToWheelVelocities(driveSignal.getVel(), TRACK_WIDTH,
                chassisLength);

        setMotorVelocities(velocities.get(0), velocities.get(1), velocities.get(2));
        setSwivelAngle(TrikeKinematics.robotToSwivelAngle(driveSignal.getVel(), chassisLength));

        driveVelocity = driveSignal.getVel();
    }

    public void setDriveVelocity(@NonNull Pose2d driveVelocity) {
        useMotorPowers = false;
        this.driveVelocity = driveVelocity;
        List<Double> velocities = TrikeKinematics.robotToWheelVelocities(driveVelocity, TRACK_WIDTH, chassisLength);
        setMotorVelocities(velocities.get(0), velocities.get(1), velocities.get(2));
        if (!approxEquals(velocities.get(0), 0) ||
                !approxEquals(velocities.get(1), 0) ||
                !approxEquals(velocities.get(2), 0))
            setSwivelAngle(TrikeKinematics.robotToSwivelAngle(driveVelocity, chassisLength));
    }

    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        useMotorPowers = true;
        List<Double> powers = TrikeKinematics.robotToWheelVelocities(drivePower, TRACK_WIDTH, chassisLength);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2));
        if (!approxEquals(powers.get(0), 0) ||
                !approxEquals(powers.get(1), 0) ||
                !approxEquals(powers.get(2), 0))
            setSwivelAngle(TrikeKinematics.robotToSwivelAngle(drivePower, chassisLength));
    }

    public void setDrivePowerSafe(Pose2d drivePower) {
        useMotorPowers = false;
        if (!rollAntiTipPID.onTarget())
            setDriveVelocity(new Pose2d(pitchCorrection, 0, rollCorrection));
        else
            setDriveVelocity(drivePower);
    }

    private static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new TankVelocityConstraint(maxVel, TRACK_WIDTH)));
    }

    private static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    // ----------------------------------------------------------------------------------------------
    // Getters And Setters
    // ----------------------------------------------------------------------------------------------

    public void setLeftVelocity(double left) {
        this.targetLeftVelocity = left;
    }

    public void setRightVelocity(double right) {
        this.targetRightVelocity = right;
    }

    public void setSwerveVelocity(double swerve) {
        this.targetSwerveVelocity = swerve;
    }

    public void setMotorVelocities(double left, double right, double swerve) {
        useMotorPowers = false;
        this.targetLeftVelocity = left;
        this.targetRightVelocity = right;
        this.targetSwerveVelocity = swerve;
    }

    public void setMotorPowers(double left, double right, double swerve) {
        useMotorPowers = true;
        this.leftPower = left;
        this.rightPower = right;
        this.swervePower = swerve;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    @Override
    public double getChassisLength() {
        return chassisLength;
    }

    public void setChassisLength(double targetChassisLength) {
        this.targetChassisLength = targetChassisLength;
        chassisLengthPID.setSetpoint(targetChassisLength);
    }

    public boolean chassisLengthOnTarget() {
        chassisLengthOnTarget = chassisLengthPID.onTarget();
        return chassisLengthOnTarget;
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
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                leftPosition,
                rightPosition,
                swervePosition);
    }

    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                leftVelocity,
                rightVelocity,
                swerveVelocity);
    }

    public void setDuckSpinnerPower(double duckSpinnerPower) {
        this.duckSpinnerPower = duckSpinnerPower;
    }

    public void toggleDuckSpinner(int mod) {
        duckSpinnerToggled = !duckSpinnerToggled;
        if (duckSpinnerToggled)
            duckSpinnerPower = DUCK_SPINNER_POWER * mod;
        else
            duckSpinnerPower = 0;
    }

    public double getSwivelAngle() {
        return swivelAngle;
    }

    @Override
    public void setSwivelAngle(double targetSwivelAngle) {
        this.targetSwivelAngle = targetSwivelAngle;
    }

    public void setMaintainChassisLengthEnabled(boolean maintainChassisLengthEnabled) {
        this.maintainChassisLengthEnabled = maintainChassisLengthEnabled;
    }

    public double getTargetChassisLength() {
        return targetChassisLength;
    }

    public void setChassisLengthMode(ChassisLengthMode chassisLengthMode) {
        this.chassisLengthMode = chassisLengthMode;
    }
    public double getVoltage() {
        return compensatedBatteryVoltage;
    }

    public void setMaintainHeadingEnabled(boolean maintainHeadingEnabled) {
        this.maintainHeadingEnabled = maintainHeadingEnabled;
    }

    public boolean isMaintainHeadingEnabled() {
        return maintainHeadingEnabled;
    }

    public void setMaintainHeading(double maintainHeading) {
        this.maintainHeading = maintainHeading;
    }

    public double getMaintainHeading() {
        return maintainHeading;
    }

    public void setUseAutonChassisLengthPID(boolean useAutonChassisLengthPID) {
        this.useAutonChassisLengthPID = useAutonChassisLengthPID;
    }
}
