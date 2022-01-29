package com.SCHSRobotics.HAL9001.system.robot.localizer;

import static java.lang.Math.PI;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.Encoder;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A localizer that uses three dead wheels to determine position.
 * <p>
 * Creation Date: 1/8/21
 *
 * @author Roadrunner Quickstart (Noah from the FTC discord); Cole Savage, Level Up
 * @version 1.0.0
 * @see Localizer
 * @see ThreeTrackingWheelLocalizer
 * @since 1.1.1
 */
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    //The names of all the encoders.
    private final String LEFT_WHEEL, RIGHT_WHEEL, PERPENDICULAR_WHEEL;
    //A map relating names to encoder objects.
    private final Map<String, Encoder> encoders = new HashMap<>();
    //The hardware constraints for the tracking wheels.
    private final TrackingWheelConfig trackingWheelConfig;
    //Weights that are applied to the perpendicular and parallel wheels, respectively.
    private double PERPENDICULAR_WEIGHT = 1, PARALLEL_WEIGHT = 1;

    /**
     * The constructor for ThreeWheelLocalizer.
     *
     * @param robot                  The robot.
     * @param leftWheel              The left encoder wheel config name.
     * @param leftWheelPose          The left encoder wheel position on the robot.
     * @param rightWheel             The right encoder wheel config name.
     * @param rightWheelPose         The right encoder wheel position on the robot.
     * @param perpendicularWheel     The perpendicular encoder wheel config name.
     * @param perpendicularWheelPose The perpendicular encoder wheel position on the robot.
     * @param trackingWheelConfig    The hardware constraints for the tracking wheels.
     */
    public ThreeWheelLocalizer(@NotNull Robot robot, String leftWheel, Pose2d leftWheelPose, String rightWheel, Pose2d rightWheelPose, String perpendicularWheel, Pose2d perpendicularWheelPose, TrackingWheelConfig trackingWheelConfig) {
        super(Arrays.asList(
                CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(leftWheelPose),
                CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(rightWheelPose),
                CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(perpendicularWheelPose)
        ));

        setPoseEstimate(new Pose2d(0, 0, 0));

        LEFT_WHEEL = leftWheel;
        RIGHT_WHEEL = rightWheel;
        PERPENDICULAR_WHEEL = perpendicularWheel;

        encoders.put(leftWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, leftWheel)));
        encoders.put(rightWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, rightWheel)));
        encoders.put(perpendicularWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, perpendicularWheel)));

        this.trackingWheelConfig = trackingWheelConfig;
    }

    /**
     * The constructor for ThreeWheelLocalizer.
     *
     * @param robot                  The robot.
     * @param leftWheel              The left encoder wheel config name.
     * @param leftWheelPose          The left encoder wheel position on the robot.
     * @param rightWheel             The right encoder wheel config name.
     * @param rightWheelPose         The right encoder wheel position on the robot.
     * @param perpendicularWheel     The perpendicular encoder wheel config name.
     * @param perpendicularWheelPose The perpendicular encoder wheel position on the robot.
     * @param distanceUnit           The units for the wheel x and y coordinates.
     * @param angleUnit              The units for the wheel orientations.
     * @param trackingWheelConfig    The hardware constraints for the tracking wheels.
     */
    public ThreeWheelLocalizer(@NotNull Robot robot, String leftWheel, Pose2d leftWheelPose, String rightWheel, Pose2d rightWheelPose, String perpendicularWheel, Pose2d perpendicularWheelPose, HALDistanceUnit distanceUnit, HALAngleUnit angleUnit, TrackingWheelConfig trackingWheelConfig) {
        this(robot, leftWheel, new Pose2d(
                HALDistanceUnit.convert(leftWheelPose.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(leftWheelPose.getY(), distanceUnit, HALDistanceUnit.INCHES), angleUnit.convertTo(HALAngleUnit.RADIANS).apply(leftWheelPose.getHeading())
        ), rightWheel, new Pose2d(
                HALDistanceUnit.convert(rightWheelPose.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(rightWheelPose.getY(), distanceUnit, HALDistanceUnit.INCHES), angleUnit.convertTo(HALAngleUnit.RADIANS).apply(rightWheelPose.getHeading())
        ), perpendicularWheel, new Pose2d(
                HALDistanceUnit.convert(perpendicularWheelPose.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(perpendicularWheelPose.getY(), distanceUnit, HALDistanceUnit.INCHES), angleUnit.convertTo(HALAngleUnit.RADIANS).apply(perpendicularWheelPose.getHeading())
        ), trackingWheelConfig);
    }

    /**
     * The constructor for ThreeWheelLocalizer.
     *
     * @param robot                 The robot.
     * @param lateralDistanceInches The lateral distance between the parallel wheels in inches.
     * @param forwardOffsetInches   How far the perpendicular wheel is offset from the parallel wheels, in inches.
     * @param leftWheel             The left encoder wheel config name.
     * @param rightWheel            The right encoder wheel config name.
     * @param perpendicularWheel    The perpendicular encoder wheel config name.
     * @param trackingWheelConfig   The hardware constraints for the tracking wheels.
     */
    public ThreeWheelLocalizer(Robot robot, double lateralDistanceInches, double forwardOffsetInches, String leftWheel, String rightWheel, String perpendicularWheel, TrackingWheelConfig trackingWheelConfig) {
        this(
                robot,
                leftWheel,
                new Pose2d(-lateralDistanceInches / 2, 0, 0),
                rightWheel,
                new Pose2d(lateralDistanceInches / 2, 0, 0),
                perpendicularWheel,
                new Pose2d(0, forwardOffsetInches, Math.toRadians(90)),
                trackingWheelConfig
        );
    }

    /**
     * The constructor for ThreeWheelLocalizer.
     *
     * @param robot               The robot.
     * @param lateralDistance     The lateral distance between the parallel wheels.
     * @param lateralDistanceUnit The units for the lateralDistance parameter.
     * @param forwardOffset       How far the perpendicular wheel is offset from the parallel wheels.
     * @param forwardOffsetUnit   The units for the forwardOffset parameter.
     * @param leftWheel           The left encoder wheel config name.
     * @param rightWheel          The right encoder wheel config name.
     * @param perpendicularWheel  The perpendicular encoder wheel config name.
     * @param trackingWheelConfig The hardware constraints for the tracking wheels.
     */
    public ThreeWheelLocalizer(Robot robot, double lateralDistance, HALDistanceUnit lateralDistanceUnit, double forwardOffset, HALDistanceUnit forwardOffsetUnit, String leftWheel, String rightWheel, String perpendicularWheel, TrackingWheelConfig trackingWheelConfig) {
        this(robot, HALDistanceUnit.convert(lateralDistance, lateralDistanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(forwardOffset, forwardOffsetUnit, HALDistanceUnit.INCHES), leftWheel, rightWheel, perpendicularWheel, trackingWheelConfig);
    }

    /**
     * Set the parallel and perpendicular weights.
     *
     * @param parallelWeight      The desired parallel weight.
     * @param perpendicularWeight The desired perpendicular weight.
     * @return This localizer.
     */
    public ThreeWheelLocalizer setWeights(double parallelWeight, double perpendicularWeight) {
        PARALLEL_WEIGHT = parallelWeight;
        PERPENDICULAR_WEIGHT = perpendicularWeight;
        return this;
    }

    /**
     * Reverse one of the encoders.
     *
     * @param encoder The name of the encoder to reverse.
     * @return This localizer.
     */
    public ThreeWheelLocalizer reverseEncoder(String encoder) {
        Encoder enc = encoders.get(encoder);
        enc.setDirection(enc.getDirection() == Encoder.Direction.FORWARD ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        return this;
    }

    /**
     * Sets the direction of one of the encoders.
     *
     * @param encoder          The name of the encoder.
     * @param encoderDirection The desired direction of the encoder.
     * @return This localizer.
     */
    public ThreeWheelLocalizer setEncoderDirection(String encoder, Encoder.Direction encoderDirection) {
        encoders.get(encoder).setDirection(encoderDirection);
        return this;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                trackingWheelConfig.encoderTicksToInches(encoders.get(LEFT_WHEEL).getCurrentPosition()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(RIGHT_WHEEL).getCurrentPosition()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(PERPENDICULAR_WHEEL).getCurrentPosition()) * PERPENDICULAR_WEIGHT
        );
    }

    @NotNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                trackingWheelConfig.encoderTicksToInches(encoders.get(LEFT_WHEEL).getCorrectedVelocity()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(RIGHT_WHEEL).getCorrectedVelocity()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(PERPENDICULAR_WHEEL).getCorrectedVelocity()) * PERPENDICULAR_WEIGHT
        );
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        Pose2d rawPose = super.getPoseEstimate();
        return new Pose2d(rawPose.getX(), rawPose.getY(), rawPose.getHeading() - PI / 2);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d value) {
        super.setPoseEstimate(new Pose2d(value.getX(), value.getY(), value.getHeading() + PI / 2));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return CoordinateMode.ROADRUNNER.convertTo(CoordinateMode.HAL).apply(super.getPoseVelocity());
    }

    @Override
    public void setPoseVelocity(@Nullable Pose2d pose2d) {
        super.setPoseVelocity(CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(pose2d));
    }
}
