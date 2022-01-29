package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.TrikeDrive;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.TrikeKinematics;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DcMotorExSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.DistanceSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.simulation.VoltageSensorSim;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.CanvasUtils;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
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

@Config
public class DriveTrain extends TrikeDrive implements Subsystem {
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0.01, 0, 0.01);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.01, 0, 0);

    public static PIDCoefficients SWIVEL_PID_COEFFICIENTS = new PIDCoefficients(0.03, 0, 0.08);
    public static PIDCoefficients CHASSIS_LENGTH_PID_COEFFICIENTS = new PIDCoefficients(0.2, 0,  0.3);
    public static double SWIVEL_PID_TOLERANCE = 10;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public static String TELEMETRY_NAME = "Drive Train";
    public static double VELOCITY_SCALE = 3.0;
    public static String AXLE_STROKE_COLOR = "Black";
    public static String WHEEL_STROKE_COLOR = "SpringGreen";

    public TrajectorySequenceRunner trajectorySequenceRunner;
    private TrajectoryFollower follower;

    private List<DcMotorEx> motors;
    private DcMotorEx leftMotor, rightMotor, swerveMotor, swivelMotor; // swerveMotor drives the module, swivelMotor rotates the module
    private DcMotorEx duckSpinner;
    private BNO055IMU imu;

    private DistanceSensor chassisLengthDistanceSensor;
    private VoltageSensor batteryVoltageSensor;

    private PIDController swivelPID, chassisLengthPID;

    private boolean simulated;

    // state
    private double leftPosition, rightPosition, swervePosition, swivelPosition;
    private double swivelAngle, targetSwivelAngle;
    private double leftVelocity, rightVelocity, swerveVelocity;
    private double targetLeftVelocity, targetRightVelocity, targetSwerveVelocity, swivelPower, duckSpinnerPower;
    private double chassisLength, targetChassisLength;
    private boolean chassisLengthOnTarget;
    private double heading, angularVelocity;
    private double angularAcceleration;
    private Pose2d driveVelocity, lastDriveVelocity;

    private long lastLoopTime, loopTime;

    private boolean maintainChassisLengthEnabled, antiTippingEnabled, smoothingEnabled;

    public DriveTrain(HardwareMap hardwareMap, boolean simulated) {
        super(TRACK_WIDTH, simulated);
        this.simulated = simulated;
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

        driveVelocity = new Pose2d(0, 0, 0);
        lastDriveVelocity = new Pose2d(0, 0, 0);
        lastLoopTime = System.nanoTime();
    }

    private double getSwivelAngleCorrection() {
        //initialization of the PID calculator's output range, target value and multipliers
        swivelPID.setOutputRange(-1.0, 1.0);
        swivelPID.setPID(SWIVEL_PID_COEFFICIENTS);
        swivelPID.setSetpoint(targetSwivelAngle);
        swivelPID.setTolerance(SWIVEL_PID_TOLERANCE);
        swivelPID.enable();

        //initialization of the PID calculator's input range and current value
        swivelPID.setInputRange(0, 360);
        swivelPID.setContinuous(true);
        swivelPID.setInput(swivelAngle);

        //calculates the angular correction to apply
        return swivelPID.performPID();
    }

    private double getChassisLengthCorrection() {
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

    public void drawFieldOverlay(Canvas canvas) {
        Pose2d pose = getPoseEstimate();

        // calculating wheel positions
        Vector2d position = pose.vec();
        Vector2d leftWheel = new Vector2d(0, Constants.TRACK_WIDTH / 2);
        Vector2d rightWheel = new Vector2d(0, -Constants.TRACK_WIDTH / 2);
        Vector2d swerveWheel = new Vector2d(-chassisLength, 0);

        // calculating wheel vectors
        List<Double> wheelVelocities = getWheelVelocities();

        Vector2d leftWheelEnd = leftWheel.plus(new Vector2d(VELOCITY_SCALE * wheelVelocities.get(0), 0));
        Vector2d rightWheelEnd = rightWheel.plus(new Vector2d(VELOCITY_SCALE * wheelVelocities.get(1), 0));
        Vector2d swerveWheelEnd = swerveWheel.plus(new Vector2d(VELOCITY_SCALE * wheelVelocities.get(2), 0).rotated(swivelAngle));

        // rotating points by heading, translating by position
        double heading = pose.getHeading();
        leftWheel = position.plus(leftWheel).rotated(heading - Math.PI / 2);
        rightWheel = position.plus(rightWheel).rotated(heading - Math.PI / 2);
        swerveWheel = position.plus(swerveWheel).rotated(heading - Math.PI / 2);
        leftWheelEnd = position.plus(leftWheelEnd).rotated(heading - Math.PI / 2);
        rightWheelEnd = position.plus(rightWheelEnd).rotated(heading - Math.PI / 2);
        swerveWheelEnd = position.plus(swerveWheelEnd).rotated(heading - Math.PI / 2);

        // drawing axles
        canvas.setStroke(AXLE_STROKE_COLOR);
        CanvasUtils.drawLine(canvas, leftWheel, rightWheel); // front axle
        CanvasUtils.drawLine(canvas, position, swerveWheel); // slinky

        // drawing wheels
        canvas.setStroke(WHEEL_STROKE_COLOR);
        CanvasUtils.drawLine(canvas, leftWheel, leftWheelEnd);
        CanvasUtils.drawLine(canvas, rightWheel, rightWheelEnd);
        CanvasUtils.drawLine(canvas, swerveWheel, swerveWheelEnd);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        updatePoseEstimate();

        if(trajectorySequenceRunner.isBusy()) {
            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity(), fieldOverlay);
            if (signal != null) setDriveSignal(signal);
        }

        leftVelocity = encoderTicksToInches(leftMotor.getVelocity());
        rightVelocity = encoderTicksToInches(rightMotor.getVelocity());
        swerveVelocity = encoderTicksToInches(swerveMotor.getVelocity());

        // motor positions/velocities
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
        }

        heading = imu.getAngularOrientation().firstAngle;
        angularVelocity = -imu.getAngularVelocity().xRotationRate;

        // PIDs
        swivelAngle = UtilMethods.wrapAngleRad(swivelMotor.getCurrentPosition() / SWIVEL_TICKS_PER_REVOLUTION * 2 * Math.PI);
        swivelPower = getSwivelAngleCorrection();

        chassisLength = chassisLengthDistanceSensor.getDistance(DistanceUnit.INCH);
        if(maintainChassisLengthEnabled) {
            double chassisLengthCorrection = getChassisLengthCorrection();
            targetLeftVelocity += chassisLengthCorrection;
            targetRightVelocity += chassisLengthCorrection;
        }
        chassisLengthOnTarget = chassisLengthPID.onTarget();

        leftMotor.setVelocity(inchesToEncoderTicks(targetLeftVelocity));
        rightMotor.setVelocity(inchesToEncoderTicks(targetRightVelocity));
        swerveMotor.setVelocity(inchesToEncoderTicks(targetSwerveVelocity));
        swivelMotor.setPower(swivelPower);
        duckSpinner.setPower(duckSpinnerPower);

        drawFieldOverlay(fieldOverlay);

        long loopClockTime = System.nanoTime();
        loopTime = loopClockTime - lastLoopTime;
        lastLoopTime = loopClockTime;
    }

    @Override
    public void stop() {
        targetLeftVelocity = 0;
        targetRightVelocity = 0;
        targetSwerveVelocity = 0;
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

            telemetryMap.put("left position", leftPosition);
            telemetryMap.put("right position", rightPosition);
            telemetryMap.put("swerve position", swervePosition);

            telemetryMap.put("swivel angle", swivelAngle);
            telemetryMap.put("target swivel angle", targetSwivelAngle);

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

            telemetryMap.put("angular velocity", Math.toDegrees(angularVelocity));
            telemetryMap.put("angular acceleration", Math.toDegrees(angularAcceleration));

            telemetryMap.put("drive velocity", driveVelocity.toString());
            telemetryMap.put("last drive velocity", lastDriveVelocity.toString());

            telemetryMap.put("loopTime", loopTime / 1e9);

            telemetryMap.put("maintain chassis length enabled", maintainChassisLengthEnabled);
            telemetryMap.put("anti tipping enabled", antiTippingEnabled);
            telemetryMap.put("smoothing enabled", smoothingEnabled);
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

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

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


    public void setMotorVelocities(double left, double right, double swerve) {
        this.targetLeftVelocity = left;
        this.targetRightVelocity = right;
        this.targetSwerveVelocity = swerve;
    }

    public void setLeftVelocity(double left) {
        this.targetLeftVelocity = left;
    }

    public void setRightVelocity(double right) {
        this.targetRightVelocity = right;
    }

    public void setSwerveVelocity(double swerve) {
        this.targetSwerveVelocity = swerve;
    }

    @Override
    public double getChassisLength() {
        return chassisLength;
    }

    public boolean chassisLengthOnTarget() {
        return chassisLengthOnTarget;
    }

    public void setChassisLength(double targetChassisLength) {
        this.targetChassisLength = targetChassisLength;
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

    public boolean isSmoothingEnabled() {
        return smoothingEnabled;
    }

    public void setSmoothingEnabled(boolean smoothingEnabled) {
        this.smoothingEnabled = smoothingEnabled;
    }
}
