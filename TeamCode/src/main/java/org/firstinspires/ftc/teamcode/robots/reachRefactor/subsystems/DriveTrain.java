package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.TrikeDrive;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.TrikeKinematics;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.VoltageSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants.*;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

@Config
public class DriveTrain extends TrikeDrive implements Subsystem {
    private static String TELEMETRY_NAME = "Drive Train";

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0.01, 0, 0.01);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0.01, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.01, 0, 0);

    public static PIDCoefficients SWIVEL_PID_COEFFICIENTS = new PIDCoefficients(1, 0, 0.08);
    public static PIDCoefficients CHASSIS_LENGTH_PID_COEFFICIENTS = new PIDCoefficients(0, 0,  0);
    public static double CHASSIS_LENGTH_PID_TOLERANCE = 1;
    public static double SWIVEL_PID_TOLERANCE = 1;

    public TrajectorySequenceRunner trajectorySequenceRunner;
    private TrajectoryFollower follower;

    private List<DcMotorEx> motors;
    private DcMotorEx leftMotor, rightMotor, swerveMotor, swivelMotor; // swerveMotor drives the module, swivelMotor rotates the module
    private DcMotorEx duckSpinner;
    private BNO055IMU imu;

    private DistanceSensor chassisLengthDistanceSensor;
    private VoltageSensor batteryVoltageSensor;

    private PIDController swivelPID, chassisLengthPID;

    private double leftPosition, rightPosition, swervePosition, swivelPosition;
    private double swivelAngle, targetSwivelAngle;
    private double leftVelocity, rightVelocity, swerveVelocity;
    private double targetLeftVelocity, targetRightVelocity, targetSwerveVelocity, swivelPower, duckSpinnerPower;
    private double chassisLength, targetChassisLength, chassisLengthCorrection;
    private boolean chassisLengthOnTarget, duckSpinnerToggled, simulated;
    private double heading, angularVelocity;
    private double compensatedBatteryVoltage;

    private Pose2d driveVelocity, lastDriveVelocity;

    private long lastLoopTime, loopTime;

    private boolean maintainChassisLengthEnabled, antiTippingEnabled;

    public DriveTrain(HardwareMap hardwareMap, boolean simulated) {
        super(TRACK_WIDTH, simulated);
        this.simulated = simulated;
        follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID, new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        if(simulated) {
            chassisLengthDistanceSensor = new DistanceSensorSim(MIN_CHASSIS_LENGTH - (DISTANCE_SENSOR_TO_FRONT_AXLE + DISTANCE_TARGET_TO_BACK_WHEEL));
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

                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                compensatedBatteryVoltage = batteryVoltageSensor.getVoltage();
                PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                        MOTOR_VELOCITY_PID.p, MOTOR_VELOCITY_PID.i, MOTOR_VELOCITY_PID.d,
                        MOTOR_VELOCITY_PID.f * 12 / compensatedBatteryVoltage
                );
                motor.setVelocityPIDFCoefficients(
                        compensatedCoefficients.p,
                        compensatedCoefficients.i,
                        compensatedCoefficients.d,
                        compensatedCoefficients.f
                );
            }

            swivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            swivelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        swivelPID = new PIDController(SWIVEL_PID_COEFFICIENTS);
        swivelPID.setInputRange(0, Math.toRadians(360));
        swivelPID.setContinuous(true);
        swivelPID.enable();

        chassisLengthPID = new PIDController(CHASSIS_LENGTH_PID_COEFFICIENTS);
        chassisLengthPID.setInputRange(Constants.MIN_CHASSIS_LENGTH, Constants.MAX_CHASSIS_LENGTH);
        chassisLengthPID.setOutputRange(-100, 100);
        chassisLengthPID.enable();

        driveVelocity = new Pose2d(0, 0, 0);
        lastDriveVelocity = new Pose2d(0, 0, 0);
        lastLoopTime = System.nanoTime();
    }

    private double getSwivelAngleCorrection() {
        swivelPID.setPID(SWIVEL_PID_COEFFICIENTS);
        swivelPID.setTolerance(SWIVEL_PID_TOLERANCE);

        swivelPID.setInput(swivelAngle);
        swivelPID.setSetpoint(targetSwivelAngle);

        return swivelPID.performPID();
    }

    private double getChassisLengthCorrection() {
        // returning 0 if distance sensor is likely blocked
        if(chassisLength < MIN_CHASSIS_LENGTH)
            return 0;

        chassisLengthPID.setPID(CHASSIS_LENGTH_PID_COEFFICIENTS);
        chassisLengthPID.setTolerance(CHASSIS_LENGTH_PID_TOLERANCE);

        chassisLengthPID.setInput(chassisLength);
        chassisLengthPID.setSetpoint(targetChassisLength);

        return chassisLengthPID.performPID();
    }

    @Override
    public void update(Canvas fieldOverlay) {
        updatePoseEstimate();

        if(trajectorySequenceRunner.isBusy()) {
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity(), fieldOverlay);
            if (signal != null) setDriveSignal(signal);
        }

        // updating velocity PIDs from dashboard
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                MOTOR_VELOCITY_PID.p, MOTOR_VELOCITY_PID.i, MOTOR_VELOCITY_PID.d,
                MOTOR_VELOCITY_PID.f * 12 / compensatedBatteryVoltage
        );
        for(DcMotorEx motor: motors) {
            motor.setVelocityPIDFCoefficients(
                    compensatedCoefficients.p,
                    compensatedCoefficients.i,
                    compensatedCoefficients.d,
                    compensatedCoefficients.f
            );
        }

        // sensor readings
        leftVelocity = encoderTicksToInches(leftMotor.getVelocity());
        rightVelocity = encoderTicksToInches(rightMotor.getVelocity());
        swerveVelocity = encoderTicksToInches(swerveMotor.getVelocity());
        if(simulated) {
            double dt = loopTime / 1e9;
            leftPosition += leftVelocity * dt;
            rightPosition += rightVelocity * dt;
            swervePosition += swerveVelocity * dt;
            swivelPosition += swivelPower * dt;
        } else {
            leftPosition = encoderTicksToInches(leftMotor.getCurrentPosition());
            rightPosition = encoderTicksToInches(rightMotor.getCurrentPosition());
            swervePosition = encoderTicksToInches(swerveMotor.getCurrentPosition());
            swivelPosition = swivelMotor.getCurrentPosition();
            swivelAngle = UtilMethods.wrapAngleRad(swivelPosition / SWIVEL_TICKS_PER_REVOLUTION * Math.toRadians(360));
        }
        heading = imu.getAngularOrientation().firstAngle;
        angularVelocity = -imu.getAngularVelocity().xRotationRate;


        // swerve optimizations
        double setpointAngle = UtilMethods.closestAngle(swivelAngle, targetSwivelAngle);
        double setpointAngleFlipped = UtilMethods.closestAngle(swivelAngle, targetSwivelAngle + Math.toRadians(180));
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)) {
            // unflip the motor direction use the setpoint
            targetSwivelAngle = UtilMethods.wrapAngle(swivelAngle + setpointAngle);
        }
        // if the closest angle to setpoint + 180 is shorter
        else {
            // flip the motor direction and use the setpoint + 180
            targetSwerveVelocity *= -1;
            targetSwivelAngle = UtilMethods.wrapAngle(swivelAngle + setpointAngleFlipped);
        }
        if(simulated)
            swivelAngle = targetSwivelAngle;

        // PIDs
        swivelPower = getSwivelAngleCorrection();
        chassisLength = chassisLengthDistanceSensor.getDistance(DistanceUnit.INCH) + DISTANCE_SENSOR_TO_FRONT_AXLE + DISTANCE_TARGET_TO_BACK_WHEEL;
        if(maintainChassisLengthEnabled && !simulated) {
            chassisLengthCorrection = getChassisLengthCorrection();
            targetLeftVelocity += chassisLengthCorrection;
            targetRightVelocity += chassisLengthCorrection;
        }
        chassisLengthOnTarget = chassisLengthPID.onTarget();

        leftMotor.setVelocity(inchesToEncoderTicks(targetLeftVelocity));
        rightMotor.setVelocity(inchesToEncoderTicks(targetRightVelocity));
        swerveMotor.setVelocity(inchesToEncoderTicks(targetSwerveVelocity));
        swivelMotor.setPower(swivelPower);
        duckSpinner.setPower(duckSpinnerPower);

        lastDriveVelocity = driveVelocity;
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate(), chassisLength, swivelAngle, getWheelVelocities());

        long loopClockTime = System.nanoTime();
        loopTime = loopClockTime - lastLoopTime;
        lastLoopTime = loopClockTime;
    }

    @Override
    public void stop() {
        targetLeftVelocity = 0;
        targetRightVelocity = 0;
        targetSwerveVelocity = 0;
        swivelPower = 0;
        duckSpinnerPower = 0;
        chassisLengthPID.disable();
        swivelPID.disable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();

        if(debug) {
            telemetryMap.put("x", getPoseEstimate().getX());
            telemetryMap.put("y", getPoseEstimate().getY());
            telemetryMap.put("heading", Math.toDegrees(getPoseEstimate().getHeading()));

            if(trajectorySequenceRunner.isBusy()) {
                telemetryMap.put("xError", trajectorySequenceRunner.getLastPoseError().getX());
                telemetryMap.put("yError", trajectorySequenceRunner.getLastPoseError().getY());
                telemetryMap.put("headingError", Math.toDegrees(trajectorySequenceRunner.getLastPoseError().getHeading()));
            }

            telemetryMap.put("left position", inchesToEncoderTicks(leftPosition));
            telemetryMap.put("right position", inchesToEncoderTicks(rightPosition));
            telemetryMap.put("swerve position", inchesToEncoderTicks(swervePosition));

            telemetryMap.put("swivel angle", Math.toDegrees(swivelAngle));
            telemetryMap.put("target swivel angle", Math.toDegrees(targetSwivelAngle));

            telemetryMap.put("left velocity", leftVelocity);
            telemetryMap.put("right velocity",rightVelocity);
            telemetryMap.put("swerve velocity", swerveVelocity);

            telemetryMap.put("target left velocity", targetLeftVelocity);
            telemetryMap.put("target right velocity", targetRightVelocity);
            telemetryMap.put("target swerve velocity", targetSwerveVelocity);
            telemetryMap.put("swivel power", swivelPower);
            telemetryMap.put("duck spinner power", duckSpinnerPower);

            telemetryMap.put("chassis length", chassisLength);
            telemetryMap.put("target chassis length", targetChassisLength);
            telemetryMap.put("chassis length PID on target", chassisLengthOnTarget);
            telemetryMap.put("chassis length PID correction", chassisLengthCorrection);

            telemetryMap.put("angular velocity", Math.toDegrees(angularVelocity));

            telemetryMap.put("drive velocity", driveVelocity.toString());
            telemetryMap.put("last drive velocity", lastDriveVelocity.toString());

            telemetryMap.put("loopTime", loopTime / 1e9);

            telemetryMap.put("maintain chassis length enabled", maintainChassisLengthEnabled);
            telemetryMap.put("anti tipping enabled", antiTippingEnabled);
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    //----------------------------------------------------------------------------------------------
    // Trajectory Following
    //----------------------------------------------------------------------------------------------

    @Override
    public void setDriveSignal(@NonNull DriveSignal driveSignal) {
        List<Double> velocities = TrikeKinematics.robotToWheelVelocities(driveSignal.getVel(), TRACK_WIDTH, getChassisLength());

        setMotorVelocities(velocities.get(0), velocities.get(1), velocities.get(2));
        setSwivelAngle(TrikeKinematics.robotToSwivelAngle(driveSignal.getVel(), getChassisLength()));

        driveVelocity = driveSignal.getVel();
    }

    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        List<Double> velocities = TrikeKinematics.robotToWheelVelocities(drivePower, TRACK_WIDTH, getChassisLength());
        setMotorVelocities(velocities.get(0), velocities.get(1), velocities.get(2));
        setSwivelAngle(TrikeKinematics.robotToSwivelAngle(drivePower, getChassisLength()));
    }

    private static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new TankVelocityConstraint(maxVel, trackWidth)
        ));
    }

    private static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Supplier<Pose2d> poseProvider) {
        return new TrajectorySequenceBuilder(
                poseProvider.get(),
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

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
        this.targetLeftVelocity = left;
        this.targetRightVelocity = right;
        this.targetSwerveVelocity = swerve;
    }

    @Override
    public double getChassisLength() {
        return chassisLength;
    }

    public void setChassisLength(double targetChassisLength) {
        this.targetChassisLength = targetChassisLength;
    }

    public boolean chassisLengthOnTarget() {
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

    public void toggleDuckSpinner(int mod) {
        duckSpinnerToggled = !duckSpinnerToggled;
        if(duckSpinnerToggled)
            duckSpinnerPower = 0.5 * mod;
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

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
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
}
