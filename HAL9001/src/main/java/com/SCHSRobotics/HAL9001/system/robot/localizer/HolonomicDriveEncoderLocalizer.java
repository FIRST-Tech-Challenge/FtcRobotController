package com.SCHSRobotics.HAL9001.system.robot.localizer;

import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.HolonomicDrivetrain;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.SCHSRobotics.HAL9001.util.misc.Timer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A localizer used for 4-wheel holonomic drivetrains. Uses ONLY the drivetrain encoders to determine position.
 * <p>
 * Creation Date: 1/10/21
 *
 * @author Roadrunner Source Code; Cole Savage, Level Up
 * @version 1.0.0
 * @see Localizer
 * @see HolonomicDrivetrain
 * @since 1.1.1
 */
public class HolonomicDriveEncoderLocalizer implements Localizer {
    //The names of the motors.
    private final String TOP_LEFT, BOT_LEFT, TOP_RIGHT, BOT_RIGHT;
    //The drivetrain using this localizer.
    private final HolonomicDrivetrain drivetrain;
    private final Timer timer = new Timer();
    //The localizer's current estimate of the drivetrain's pose.
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    //The localizer's current estimate of the pose velocity.
    private Pose2d poseVelocity = new Pose2d(0, 0, 0);
    //The last wheel positions of the drivetrain.
    private List<Double> lastWheelPositions = new ArrayList<>();
    //The drivetrain's last heading value.
    private double lastHeading = Double.NaN;


    /**
     * The constructor for HolonomicDriveEncoderLocalizer.
     *
     * @param drivetrain The drivetrain using the localizer.
     * @param topLeft    The top left motor config name.
     * @param topRight   The top right motor config name.
     * @param botLeft    The bottom left motor config name.
     * @param botRight   The bottom right motor config name.
     */
    public HolonomicDriveEncoderLocalizer(HolonomicDrivetrain drivetrain, String topLeft, String topRight, String botLeft, String botRight) {

        this.drivetrain = drivetrain;

        TOP_LEFT = topLeft;
        TOP_RIGHT = topRight;
        BOT_LEFT = botLeft;
        BOT_RIGHT = botRight;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return CoordinateMode.ROADRUNNER.convertTo(CoordinateMode.HAL).apply(poseEstimate);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        lastWheelPositions = new ArrayList<>();
        poseEstimate = CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return CoordinateMode.ROADRUNNER.convertTo(CoordinateMode.HAL).apply(poseVelocity);
    }

    @Override
    public void update() {
        List<Double> wheelPositions = Arrays.asList(
                drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorEncoderPosition(TOP_LEFT)),
                drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorEncoderPosition(BOT_LEFT)),
                drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorEncoderPosition(BOT_RIGHT)),
                drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorEncoderPosition(TOP_RIGHT))
        );
        double heading = (((wheelPositions.get(3) + wheelPositions.get(2)) - (wheelPositions.get(0) + wheelPositions.get(1))) / 4) / drivetrain.driveConfig.TRACK_WIDTH;

        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();

            for (int i = 0; i < wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                    wheelDeltas,
                    drivetrain.driveConfig.TRACK_WIDTH,
                    drivetrain.driveConfig.WHEEL_BASE,
                    drivetrain.getLateralMultiplier()
            );

            double finalHeadingDelta = Angle.normDelta(heading - lastHeading);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
        }

        List<Double> wheelVelocities = Arrays.asList(
                drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorVelocity(TOP_LEFT)),
                drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorVelocity(BOT_LEFT)),
                drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorVelocity(BOT_RIGHT)),
                drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorVelocity(TOP_RIGHT))
        );
        poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                wheelVelocities,
                drivetrain.driveConfig.TRACK_WIDTH,
                drivetrain.driveConfig.WHEEL_BASE,
                drivetrain.getLateralMultiplier()
        );

        if (!Double.isNaN(lastHeading)) {
            double headingDelta = Angle.normDelta(heading - lastHeading);
            poseVelocity = new Pose2d(poseVelocity.vec(), headingDelta / timer.getTimePassed(HALTimeUnit.SECONDS));
            timer.reset();
        }

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }
}