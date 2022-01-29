package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import com.SCHSRobotics.HAL9001.system.config.AutonomousConfig;
import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.HALTrajectory;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.HALTrajectoryBuilder;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.LynxModuleUtil;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.RoadrunnerConfig;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * A built-in HAL mecanum drive subsystem. Has roadrunner compatibility.
 * <p>
 * Creation Date: 1/5/21
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see MecanumDriveSimple
 * @see XDriveSimple
 * @see XDrive
 * @see HolonomicDrivetrain
 * @see Drivetrain
 * @see com.acmerobotics.roadrunner.drive.MecanumDrive
 * @since 1.1.1
 */
public class MecanumDrive extends MecanumDriveSimple {
    //The roadrunner configuration settings and drivetrain hardware constraints.
    public final RoadrunnerConfig rrConfig;
    //The roadrunner drivetrain class that this class acts as a partial wrapper for. Used for interfacing with roadrunner.
    private final SimpleMecanumDriveRoadrunnerController rrInterface;
    //The number of poses that the drivetrain can store in its pose history.
    private int POSE_HISTORY_LIMIT = 100;

    /**
     * The constructor for MecanumDrive.
     *
     * @param robot     The robot using this drivetrain.
     * @param rrConfig  The roadrunner config settings and drivetrain hardware constraints.
     * @param topLeft   The top left motor config name.
     * @param topRight  The top right motor config name.
     * @param botLeft   The bottom left motor config name.
     * @param botRight  The bottom right motor config name.
     * @param useConfig Whether or not the drivetrain uses the HAL config system.
     */
    public MecanumDrive(Robot robot, RoadrunnerConfig rrConfig, String topLeft, String topRight, String botLeft, String botRight, boolean useConfig) {
        super(robot, rrConfig, topLeft, topRight, botLeft, botRight, useConfig);

        this.rrConfig = rrConfig;

        rrInterface = new SimpleMecanumDriveRoadrunnerController();
        rrInterface.setLocalizer(localizer);
    }

    /**
     * The constructor for MecanumDrive.
     *
     * @param robot    The robot using this drivetrain.
     * @param rrConfig The roadrunner config settings and drivetrain hardware constraints.
     * @param topLeft  The top left motor config name.
     * @param topRight The top right motor config name.
     * @param botLeft  The bottom left motor config name.
     * @param botRight The bottom right motor config name.
     */
    public MecanumDrive(Robot robot, RoadrunnerConfig rrConfig, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, rrConfig, topLeft, topRight, botLeft, botRight, true);
    }

    @AutonomousConfig
    public static ConfigParam[] autonomousConfig() {
        return new ConfigParam[]{
                new ConfigParam("Drive Mode", DriveMode.STANDARD),
                new ConfigParam("Velocity Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Turn Speed Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Velocity Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Velocity Cap", ConfigParam.numberMap(0, 1, 0.05), 1.0),
                new ConfigParam("Turn Speed Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Turn Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Turn Speed Cap", ConfigParam.numberMap(0, 1, 0.05), 1.0),
                new ConfigParam("Turn Button Power", ConfigParam.numberMap(0, 1, 0.05), 0.3)
        };
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam("Drive Mode", DriveMode.STANDARD),
                new ConfigParam(DRIVE_STICK, Button.VectorInputs.right_stick),
                new ConfigParam(TURN_STICK, Button.DoubleInputs.left_stick_x),
                new ConfigParam(TURN_LEFT_BUTTON, Button.BooleanInputs.noButton),
                new ConfigParam(TURN_RIGHT_BUTTON, Button.BooleanInputs.noButton),
                new ConfigParam(SPEED_TOGGLE, Button.BooleanInputs.noButton),
                new ConfigParam(TURN_SPEED_TOGGLE, Button.BooleanInputs.noButton),
                new ConfigParam("Velocity Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Turn Speed Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Velocity Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Velocity Cap", ConfigParam.numberMap(0, 1, 0.05), 1.0),
                new ConfigParam("Turn Speed Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Turn Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Turn Speed Cap", ConfigParam.numberMap(0, 1, 0.05), 1.0),
                new ConfigParam("Turn Button Power", ConfigParam.numberMap(0, 1, 0.05), 0.3)
        };
    }

    /**
     * Uses roadrunner to turn to an angle asynchronously.
     *
     * @param angle The angle to turn to.
     */
    public void turnAsync(double angle) {
        rrInterface.turnAsync(angle);
    }

    /**
     * Uses roadrunner to turn to an angle asynchronously.
     *
     * @param angle     The angle to turn to.
     * @param angleUnit The unit of the angle.
     */
    public void turnAsync(double angle, @NotNull HALAngleUnit angleUnit) {
        turnAsync(angleUnit.convertTo(HALAngleUnit.RADIANS).apply(angle));
    }

    /**
     * Uses roadrunner to turn to an angle.
     *
     * @param angle The angle to turn to.
     */
    public void turn(double angle) {
        rrInterface.turn(angle);
    }

    /**
     * Uses roadrunner to turn to an angle.
     *
     * @param angle     The angle to turn to.
     * @param angleUnit The unit of the angle.
     */
    public void turn(double angle, @NotNull HALAngleUnit angleUnit) {
        turn(angleUnit.convertTo(HALAngleUnit.RADIANS).apply(angle));
    }

    /**
     * Uses roadrunner to follow a trajectory asynchronously.
     *
     * @param trajectory The trajectory to follow.
     */
    public void followTrajectoryAsync(@NotNull HALTrajectory trajectory) {
        rrInterface.followTrajectoryAsync(trajectory.toRoadrunner());
    }

    /**
     * Uses roadrunner to follow a trajectory.
     *
     * @param trajectory The trajectory to follow.
     */
    public void followTrajectory(@NotNull HALTrajectory trajectory) {
        rrInterface.followTrajectory(trajectory.toRoadrunner());
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose    The drivetrain's starting pose.
     * @param distanceUnit The units of the drivetrain's starting x and y coordinates.
     * @param angleUnit    The units of the drivetrain's starting heading.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(@NotNull Pose2d startPose, HALDistanceUnit distanceUnit, @NotNull HALAngleUnit angleUnit) {
        return new HALTrajectoryBuilder(
                new TrajectoryBuilder(
                        coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                                HALDistanceUnit.convert(startPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                                HALDistanceUnit.convert(startPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(startPose.getHeading())
                        )),
                        rrInterface.velConstraint,
                        rrInterface.accelConstraint
                ), coordinateMode, distanceUnit, angleUnit
        );
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose    The drivetrain's starting pose.
     * @param distanceUnit The units of the drivetrain's starting x and y coordinates.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(@NotNull Pose2d startPose, HALDistanceUnit distanceUnit) {
        return trajectoryBuilder(startPose, distanceUnit, HALAngleUnit.RADIANS);
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose The drivetrain's starting pose.
     * @param angleUnit The units of the drivetrain's starting heading.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(@NotNull Pose2d startPose, HALAngleUnit angleUnit) {
        return trajectoryBuilder(startPose, HALDistanceUnit.INCHES, angleUnit);
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose The drivetrain's starting pose.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return trajectoryBuilder(startPose, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS);
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose    The drivetrain's starting pose.
     * @param distanceUnit The units of the drivetrain's starting x and y coordinates.
     * @param angleUnit    The units of the drivetrain's starting heading.
     * @param reversed     Whether the trajectory is reversed.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(@NotNull Pose2d startPose, HALDistanceUnit distanceUnit, @NotNull HALAngleUnit angleUnit, boolean reversed) {
        return new HALTrajectoryBuilder(
                new TrajectoryBuilder(
                        coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                                HALDistanceUnit.convert(startPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                                HALDistanceUnit.convert(startPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(startPose.getHeading())
                        )),
                        reversed,
                        rrInterface.velConstraint,
                        rrInterface.accelConstraint
                ), coordinateMode, distanceUnit, angleUnit
        );
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose    The drivetrain's starting pose.
     * @param distanceUnit The units of the drivetrain's starting x and y coordinates.
     * @param reversed     Whether the trajectory is reversed.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(Pose2d startPose, HALDistanceUnit distanceUnit, boolean reversed) {
        return trajectoryBuilder(startPose, distanceUnit, HALAngleUnit.RADIANS, reversed);
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose The drivetrain's starting pose.
     * @param angleUnit The units of the drivetrain's starting heading.
     * @param reversed  Whether the trajectory is reversed.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(Pose2d startPose, HALAngleUnit angleUnit, boolean reversed) {
        return trajectoryBuilder(startPose, HALDistanceUnit.INCHES, angleUnit, reversed);
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose The drivetrain's starting pose.
     * @param reversed  Whether the trajectory is reversed.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return trajectoryBuilder(startPose, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS, reversed);
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose    The drivetrain's starting pose.
     * @param startTangent The path's starting heading.
     * @param distanceUnit The units of the drivetrain's starting x and y coordinates.
     * @param angleUnit    The units of the drivetrain's starting heading and starting path tangent.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(@NotNull Pose2d startPose, double startTangent, HALDistanceUnit distanceUnit, @NotNull HALAngleUnit angleUnit) {
        return new HALTrajectoryBuilder(
                new TrajectoryBuilder(
                        coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                                HALDistanceUnit.convert(startPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                                HALDistanceUnit.convert(startPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(startPose.getHeading())
                        )),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(startTangent),
                        rrInterface.velConstraint,
                        rrInterface.accelConstraint
                ), coordinateMode, distanceUnit, angleUnit
        );
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose    The drivetrain's starting pose.
     * @param startTangent The path's starting heading.
     * @param distanceUnit The units of the drivetrain's starting x and y coordinates.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startTangent, HALDistanceUnit distanceUnit) {
        return trajectoryBuilder(startPose, startTangent, distanceUnit, HALAngleUnit.RADIANS);
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose    The drivetrain's starting pose.
     * @param startTangent The path's starting heading.
     * @param angleUnit    The units of the drivetrain's starting heading and starting path tangent.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startTangent, HALAngleUnit angleUnit) {
        return trajectoryBuilder(startPose, startTangent, HALDistanceUnit.INCHES, angleUnit);
    }

    /**
     * Generates a trajectory builder that is used to create trajectories.
     *
     * @param startPose    The drivetrain's starting pose.
     * @param startTangent The path's starting heading.
     * @return A trajectory builder that is used to create trajectories.
     */
    public HALTrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startTangent) {
        return trajectoryBuilder(startPose, startTangent, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS);
    }

    public List<Double> getWheelPositions() {
        return rrInterface.getWheelPositions();
    }

    public List<Double> getWheelVelocities() {
        return rrInterface.getWheelVelocities();
    }

    /**
     * Waits for the roadrunner interface to enter an idle state.
     */
    public void waitForRRInterfaceIdle() {
        rrInterface.waitForIdle();
    }

    /**
     * Gets whether the roadrunner interface is currently busy.
     *
     * @return Whether the roadrunner interface is currently busy.
     */
    public boolean rRInterfaceIsBusy() {
        return rrInterface.isBusy();
    }

    /**
     * Updates the roadrunner interface manually.
     */
    public void updateRRInterface() {
        rrInterface.update();
    }

    /**
     * Sets the drivetrain's pose history limit.
     *
     * @param poseHistoryLimit The drivetrain's pose history limit.
     */
    public void setPoseHistoryLimit(int poseHistoryLimit) {
        POSE_HISTORY_LIMIT = poseHistoryLimit;
    }

    @Override
    public void setLocalizer(Localizer localizer, CoordinateMode coordinateMode) {
        super.setLocalizer(localizer, coordinateMode);
        rrInterface.setLocalizer(localizer);
    }

    /**
     * The current mode of the roadrunner interface class.
     */
    private enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    /**
     * A roadrunner mecanum drive class used by MecanumDrive to interface with roadrunner.
     */
    private class SimpleMecanumDriveRoadrunnerController extends com.acmerobotics.roadrunner.drive.MecanumDrive {
        //A nanosecond timer for keeping time.
        private final NanoClock clock;
        //The trajectory velocity constraint.
        private final TrajectoryVelocityConstraint velConstraint;
        //The trajectory acceleration constraint.
        private final TrajectoryAccelerationConstraint accelConstraint;
        //The trajectory follower used to follow the trajectory.
        private final TrajectoryFollower follower;
        //The drivetrain's pose history.
        private final LinkedList<Pose2d> poseHistory;
        //The current mode of the drivetrain.
        private Mode mode;
        //The motion profile used for turning.
        private MotionProfile turnProfile;
        //The starting time for the turn motion profile.
        private double turnStart;

        /**
         * The constructor for the simple roadrunner interface.
         */
        public SimpleMecanumDriveRoadrunnerController() {
            super(rrConfig.kV, rrConfig.kA, rrConfig.kStatic, rrConfig.TRACK_WIDTH, rrConfig.WHEEL_BASE, LATERAL_MULTIPLIER);

            clock = NanoClock.system();

            mode = Mode.IDLE;

            velConstraint = new MinVelocityConstraint(Arrays.asList(
                    new AngularVelocityConstraint(rrConfig.MAX_ANG_VEL),
                    new MecanumVelocityConstraint(rrConfig.MAX_VEL, rrConfig.TRACK_WIDTH)
            ));
            accelConstraint = new ProfileAccelerationConstraint(rrConfig.MAX_ACCEL);
            follower = new HolonomicPIDVAFollower(translationCoefficients, translationCoefficients, headingCoefficients,
                    new Pose2d(rrConfig.FOLLOWER_X_TOLERANCE, rrConfig.FOLLOWER_Y_TOLERANCE, rrConfig.FOLLOWER_HEADING_TOLERANCE), rrConfig.FOLLOWER_TIMEOUT);

            poseHistory = new LinkedList<>();

            LynxModuleUtil.ensureMinimumFirmwareVersion(robot.hardwareMap);

            for (LynxModule module : robot.hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            for (DcMotorEx motor : motors.values()) {
                MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
                motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
                motor.setMotorType(motorConfigurationType);
            }

            if (rrConfig.USE_DRIVE_ENCODERS) {
                setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (rrConfig.USE_DRIVE_ENCODERS && rrConfig.MOTOR_VELO_PID != null) {
                setMotorPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rrConfig.MOTOR_VELO_PID);
            }
        }

        /**
         * Turns to an angle asynchronously.
         *
         * @param angle The angle to turn to in radians.
         */
        public void turnAsync(double angle) {
            double heading = getPoseEstimate().getHeading();

            turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(heading, 0, 0, 0),
                    new MotionState(heading + angle, 0, 0, 0),
                    rrConfig.MAX_ANG_VEL,
                    rrConfig.MAX_ANG_ACCEL
            );

            turnStart = clock.seconds();
            mode = Mode.TURN;
        }

        /**
         * Turns to an angle.
         *
         * @param angle The angle to turn to in radians.
         */
        public void turn(double angle) {
            turnAsync(angle);
            waitForIdle();
        }

        /**
         * Follows a trajectory asynchronously.
         *
         * @param trajectory The trajectory to follow.
         */
        public void followTrajectoryAsync(Trajectory trajectory) {
            follower.followTrajectory(trajectory);
            mode = Mode.FOLLOW_TRAJECTORY;
        }

        /**
         * Follows a trajectory.
         *
         * @param trajectory The trajectory to follow.
         */
        public void followTrajectory(Trajectory trajectory) {
            followTrajectoryAsync(trajectory);
            waitForIdle();
        }

        /**
         * Updates the drivetrain.
         */
        public void update() {
            updatePoseEstimate();

            Pose2d currentPose = localizerCoordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(getPoseEstimate());
            currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading());

            poseHistory.add(currentPose);

            if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
                poseHistory.removeFirst();
            }

            switch (mode) {
                case IDLE:
                    // do nothing
                    break;
                case TURN: {
                    double t = clock.seconds() - turnStart;

                    MotionState targetState = turnProfile.get(t);

                    headingController.setTargetPosition(targetState.getX());

                    double correction = headingController.update(currentPose.getHeading());

                    double targetOmega = targetState.getV();
                    double targetAlpha = targetState.getA();
                    setDriveSignal(new DriveSignal(new Pose2d(
                            0, 0, targetOmega + correction
                    ), new Pose2d(
                            0, 0, targetAlpha
                    )));

                    if (t >= turnProfile.duration()) {
                        mode = Mode.IDLE;
                        setDriveSignal(new DriveSignal());
                    }

                    break;
                }
                case FOLLOW_TRAJECTORY: {
                    setDriveSignal(follower.update(currentPose));

                    if (!follower.isFollowing()) {
                        mode = Mode.IDLE;
                        setDriveSignal(new DriveSignal());
                    }

                    break;
                }
            }
        }

        /**
         * Waits until the drivetrain is idle.
         */
        public void waitForIdle() {
            while (!Thread.currentThread().isInterrupted() && isBusy()) {
                update();
            }
        }

        /**
         * Gets whether the drivetrain is busy.
         *
         * @return Whether the drivetrain is busy.
         */
        public boolean isBusy() {
            return mode != Mode.IDLE;
        }

        @NotNull
        @Override
        public List<Double> getWheelPositions() {
            List<Double> wheelPositions = new ArrayList<>();
            for (String motor : motors.keySet()) {
                wheelPositions.add(rrConfig.encoderTicksToInches(getMotorEncoderPosition(motor)));
            }
            return wheelPositions;
        }

        @Override
        public List<Double> getWheelVelocities() {
            List<Double> wheelVelocities = new ArrayList<>();
            for (String motor : motors.keySet()) {
                wheelVelocities.add(rrConfig.encoderTicksToInches(getMotorVelocity(motor)));
            }
            return wheelVelocities;
        }

        @Override
        public void setMotorPowers(double topLeft, double botLeft, double botRight, double topRight) {
            setMotorPower(TOP_LEFT, topLeft);
            setMotorPower(BOT_LEFT, botLeft);
            setMotorPower(BOT_RIGHT, botRight);
            setMotorPower(TOP_RIGHT, topRight);
        }

        @Override
        public double getRawExternalHeading() {
            return 0;
        }
    }
}