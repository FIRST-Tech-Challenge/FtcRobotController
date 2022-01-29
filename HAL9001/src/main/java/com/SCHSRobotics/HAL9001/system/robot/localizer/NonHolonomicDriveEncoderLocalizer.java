package com.SCHSRobotics.HAL9001.system.robot.localizer;

import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.NonHolonomicDrivetrain;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.TankDriveSimple;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.SCHSRobotics.HAL9001.util.misc.Timer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A localizer used for non-holonomic drivetrains. Uses ONLY drivetrain encoders to determine position.
 * <p>
 * Creation Date: 1/10/21
 *
 * @author Roadrunner Source Code; Cole Savage, Level Up
 * @version 1.0.0
 * @see Localizer
 * @see NonHolonomicDrivetrain
 * @since 1.1.1
 */
public class NonHolonomicDriveEncoderLocalizer implements Localizer {
    //The left and right motor names.
    private final String[] leftMotors, rightMotors;
    //The drivetrain using this localizer.
    private final TankDriveSimple drivetrain;
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
     * The constructor for NonHolonomicDriveEncoderLocalizer.
     *
     * @param drivetrain  The drivetrain using this localizer.
     * @param leftMotors  The config names of the left motors.
     * @param rightMotors The config names of the right motors.
     */
    public NonHolonomicDriveEncoderLocalizer(TankDriveSimple drivetrain, @NotNull String[] leftMotors, @NotNull String[] rightMotors) {

        this.drivetrain = drivetrain;

        this.leftMotors = leftMotors.clone();
        this.rightMotors = rightMotors.clone();
    }

    /**
     * The constructor for NonHolonomicDriveEncoderLocalizer.
     *
     * @param drivetrain The drivetrain using this localizer.
     * @param leftMotor  The config name of the left motor.
     * @param rightMotor The config names of the right motor.
     */
    public NonHolonomicDriveEncoderLocalizer(TankDriveSimple drivetrain, String leftMotor, String rightMotor) {
        this(drivetrain, new String[]{leftMotor}, new String[]{rightMotor});
    }

    /**
     * The constructor for NonHolonomicDriveEncoderLocalizer.
     *
     * @param drivetrain The drivetrain using this localizer.
     * @param topLeft    The top left motor config name.
     * @param topRight   The top right motor config name.
     * @param botLeft    The bottom left motor config name.
     * @param botRight   The bottom right motor config name.
     */
    public NonHolonomicDriveEncoderLocalizer(TankDriveSimple drivetrain, String topLeft, String topRight, String botLeft, String botRight) {
        this(drivetrain, new String[]{topLeft, botLeft}, new String[]{topRight, botRight});
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

        double leftPosition = 0;
        double leftVelocity = 0;
        for (String motor : leftMotors) {
            leftPosition += drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorEncoderPosition(motor));
            leftVelocity += drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorVelocity(motor));
        }
        leftPosition /= leftMotors.length;
        leftVelocity /= leftMotors.length;

        double rightPosition = 0;
        double rightVelocity = 0;
        for (String motor : rightMotors) {
            leftPosition += drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorEncoderPosition(motor));
            leftVelocity += drivetrain.driveConfig.encoderTicksToInches(drivetrain.getMotorVelocity(motor));
        }
        rightPosition /= rightMotors.length;
        rightVelocity /= rightMotors.length;

        List<Double> wheelPositions = Arrays.asList(
                leftPosition,
                rightPosition
        );
        double heading = (wheelPositions.get(1) - wheelPositions.get(0)) / drivetrain.driveConfig.TRACK_WIDTH;
        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();

            for (int i = 0; i < wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            Pose2d robotPoseDelta = TankKinematics.wheelToRobotVelocities(
                    wheelDeltas,
                    drivetrain.driveConfig.TRACK_WIDTH
            );

            double finalHeadingDelta = Angle.normDelta(heading - lastHeading);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
        }

        List<Double> wheelVelocities = Arrays.asList(
                drivetrain.driveConfig.encoderTicksToInches(leftVelocity),
                drivetrain.driveConfig.encoderTicksToInches(rightVelocity)
        );
        poseVelocity = TankKinematics.wheelToRobotVelocities(
                wheelVelocities,
                drivetrain.driveConfig.TRACK_WIDTH
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
