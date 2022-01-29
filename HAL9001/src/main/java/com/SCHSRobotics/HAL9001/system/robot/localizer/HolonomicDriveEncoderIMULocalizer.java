package com.SCHSRobotics.HAL9001.system.robot.localizer;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.AxesSigns;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.BNO055IMUUtil;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.HolonomicDrivetrain;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A localizer used for 4-wheel holonomic drivetrains. Uses both the IMU and drivetrain encoders to determine position.
 * <p>
 * Creation Date: 1/8/21
 *
 * @author Roadrunner Source Code; Cole Savage, Level Up
 * @version 1.0.0
 * @see Localizer
 * @see HolonomicDrivetrain
 * @since 1.1.1
 */
public class HolonomicDriveEncoderIMULocalizer implements Localizer {

    private final String TOP_LEFT, TOP_RIGHT, BOT_LEFT, BOT_RIGHT;
    //The imu object.
    private final BNO055IMU imu;
    //The drivetrain using this localizer.
    private final HolonomicDrivetrain drivetrain;
    //The localizer's current estimate of the drivetrain's pose.
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    //The localizer's current estimate of the pose velocity.
    private Pose2d poseVelocity = new Pose2d(0, 0, 0);
    //The last wheel positions of the drivetrain.
    private List<Double> lastWheelPositions = new ArrayList<>();
    //The drivetrain's last heading value.
    private double lastHeading = Double.NaN;

    /**
     * The constructor for HolonomicDriveEncoderIMULocalizer.
     *
     * @param robot         The robot using the drivetrain.
     * @param drivetrain    The drivetrain using the localizer.
     * @param imu           The config name of the IMU.
     * @param imuParameters The parameters for the imu.
     * @param topLeft       The top left motor config name.
     * @param topRight      The top right motor config name.
     * @param botLeft       The bottom left motor config name.
     * @param botRight      The bottom right motor config name.
     */
    public HolonomicDriveEncoderIMULocalizer(@NotNull Robot robot, HolonomicDrivetrain drivetrain, String imu, @NotNull BNO055IMU.Parameters imuParameters, String topLeft, String topRight, String botLeft, String botRight) {
        this.drivetrain = drivetrain;

        this.imu = robot.hardwareMap.get(BNO055IMU.class, imu);

        TOP_LEFT = topLeft;
        TOP_RIGHT = topRight;
        BOT_LEFT = botLeft;
        BOT_RIGHT = botRight;

        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(imuParameters);
    }

    /**
     * The constructor for HolonomicDriveEncoderIMULocalizer.
     *
     * @param robot      The robot using the drivetrain.
     * @param drivetrain The drivetrain using the localizer.
     * @param imu        The config name of the IMU.
     * @param topLeft    The top left motor config name.
     * @param topRight   The top right motor config name.
     * @param botLeft    The bottom left motor config name.
     * @param botRight   The bottom right motor config name.
     */
    public HolonomicDriveEncoderIMULocalizer(Robot robot, HolonomicDrivetrain drivetrain, String imu, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, drivetrain, imu, new BNO055IMU.Parameters(), topLeft, topRight, botLeft, botRight);
    }

    /**
     * Remaps the imu axes.
     *
     * @param axesOrder The order of the axes.
     * @param axesSigns The signs of the axes.
     * @return This localizer.
     */
    public HolonomicDriveEncoderIMULocalizer remapIMUAxes(AxesOrder axesOrder, AxesSigns axesSigns) {
        BNO055IMUUtil.remapAxes(imu, axesOrder, axesSigns);
        return this;
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
        double heading = imu.getAngularOrientation().firstAngle;
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

        poseVelocity = new Pose2d(poseVelocity.vec(), imu.getAngularVelocity().zRotationRate);

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }
}