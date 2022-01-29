package com.SCHSRobotics.HAL9001.system.robot.localizer;

import static java.lang.Math.PI;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.AxesSigns;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.BNO055IMUUtil;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.Encoder;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A localizer that uses two dead wheels and the IMU to determine position.
 * Based on <a href="https://github.com/NoahBres/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/TwoWheelTrackingLocalizer.java">this file</a>.
 * <p>
 * Creation Date: 1/8/21
 *
 * @author Roadrunner Quickstart (Noah from the FTC discord); Cole Savage, Level Up.
 * @version 1.0.0
 * @see Localizer
 * @see TwoTrackingWheelLocalizer
 * @since 1.1.1
 */
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    //The names of all the encoders.
    private final String PARALLEL_WHEEL, PERPENDICULAR_WHEEL;
    //A map relating names to encoder objects.
    private final Map<String, Encoder> encoders = new HashMap<>();
    //The imu object.
    private final BNO055IMU imu;
    //The hardware constraints for the tracking wheels.
    private final TrackingWheelConfig trackingWheelConfig;
    //Weights that are applied to the perpendicular and parallel wheels, respectively.
    private double PARALLEL_WEIGHT = 1, PERPENDICULAR_WEIGHT = 1;

    /**
     * The constructor for TwoWheelLocalizer.
     *
     * @param robot                  The robot.
     * @param imu                    The config name of the IMU.
     * @param imuParameters          The parameters for the imu.
     * @param parallelWheel          The config name of the parallel tracking wheel.
     * @param parallelWheelPose      The parallel wheel position on the robot.
     * @param perpendicularWheel     The config name of the perpendicular tracking wheel.
     * @param perpendicularWheelPose The perpendicular wheel position on the robot.
     * @param trackingWheelConfig    The hardware constraints for the tracking wheels.
     */
    public TwoWheelLocalizer(@NotNull Robot robot, String imu, BNO055IMU.Parameters imuParameters, String parallelWheel, Pose2d parallelWheelPose, String perpendicularWheel, Pose2d perpendicularWheelPose, TrackingWheelConfig trackingWheelConfig) {
        super(Arrays.asList(
                CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(parallelWheelPose),
                CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(perpendicularWheelPose)
        ));

        setPoseEstimate(new Pose2d(0, 0, 0));

        PARALLEL_WHEEL = parallelWheel;
        PERPENDICULAR_WHEEL = perpendicularWheel;

        this.imu = robot.hardwareMap.get(BNO055IMU.class, imu);
        encoders.put(parallelWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, parallelWheel)));
        encoders.put(perpendicularWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, perpendicularWheel)));

        this.imu.initialize(imuParameters);

        this.trackingWheelConfig = trackingWheelConfig;
    }

    /**
     * The constructor for TwoWheelLocalizer.
     *
     * @param robot                  The robot.
     * @param imu                    The config name of the IMU.
     * @param imuParameters          The parameters for the imu.
     * @param parallelWheel          The config name of the parallel tracking wheel.
     * @param parallelWheelPose      The parallel wheel position on the robot.
     * @param perpendicularWheel     The config name of the perpendicular tracking wheel.
     * @param perpendicularWheelPose The perpendicular wheel position on the robot.
     * @param distanceUnit           The units for the wheel x and y coordinates.
     * @param angleUnit              The units for the wheel orientations.
     * @param trackingWheelConfig    The hardware constraints for the tracking wheels.
     */
    public TwoWheelLocalizer(@NotNull Robot robot, String imu, BNO055IMU.Parameters imuParameters, String parallelWheel, @NotNull Pose2d parallelWheelPose, String perpendicularWheel, @NotNull Pose2d perpendicularWheelPose, HALDistanceUnit distanceUnit, @NotNull HALAngleUnit angleUnit, TrackingWheelConfig trackingWheelConfig) {
        this(robot, imu, imuParameters, parallelWheel, new Pose2d(
                HALDistanceUnit.convert(parallelWheelPose.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(parallelWheelPose.getY(), distanceUnit, HALDistanceUnit.INCHES), angleUnit.convertTo(HALAngleUnit.RADIANS).apply(parallelWheelPose.getHeading())
        ), perpendicularWheel, new Pose2d(
                HALDistanceUnit.convert(perpendicularWheelPose.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(perpendicularWheelPose.getY(), distanceUnit, HALDistanceUnit.INCHES), angleUnit.convertTo(HALAngleUnit.RADIANS).apply(perpendicularWheelPose.getHeading())
        ), trackingWheelConfig);
    }

    /**
     * The constructor for TwoWheelLocalizer.
     *
     * @param robot                  The robot.
     * @param imu                    The config name of the IMU.
     * @param parallelWheel          The config name of the parallel tracking wheel.
     * @param parallelWheelPose      The parallel wheel position on the robot.
     * @param perpendicularWheel     The config name of the perpendicular tracking wheel.
     * @param perpendicularWheelPose The perpendicular wheel position on the robot.
     * @param trackingWheelConfig    The hardware constraints for the tracking wheels.
     */
    public TwoWheelLocalizer(Robot robot, String imu, String parallelWheel, Pose2d parallelWheelPose, String perpendicularWheel, Pose2d perpendicularWheelPose, TrackingWheelConfig trackingWheelConfig) {
        this(robot, imu, new BNO055IMU.Parameters(), parallelWheel, parallelWheelPose, perpendicularWheel, perpendicularWheelPose, trackingWheelConfig);
    }

    /**
     * The constructor for TwoWheelLocalizer.
     *
     * @param robot                  The robot.
     * @param imu                    The config name of the IMU.
     * @param parallelWheel          The config name of the parallel tracking wheel.
     * @param parallelWheelPose      The parallel wheel position on the robot.
     * @param perpendicularWheel     The config name of the perpendicular tracking wheel.
     * @param perpendicularWheelPose The perpendicular wheel position on the robot.
     * @param distanceUnit           The units for the wheel x and y coordinates.
     * @param angleUnit              The units for the wheel orientations.
     * @param trackingWheelConfig    The hardware constraints for the tracking wheels.
     */
    public TwoWheelLocalizer(Robot robot, String imu, String parallelWheel, @NotNull Pose2d parallelWheelPose, String perpendicularWheel, @NotNull Pose2d perpendicularWheelPose, HALDistanceUnit distanceUnit, @NotNull HALAngleUnit angleUnit, TrackingWheelConfig trackingWheelConfig) {
        this(robot, imu, parallelWheel, new Pose2d(
                HALDistanceUnit.convert(parallelWheelPose.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(parallelWheelPose.getY(), distanceUnit, HALDistanceUnit.INCHES), angleUnit.convertTo(HALAngleUnit.RADIANS).apply(parallelWheelPose.getHeading())
        ), perpendicularWheel, new Pose2d(
                HALDistanceUnit.convert(perpendicularWheelPose.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(perpendicularWheelPose.getY(), distanceUnit, HALDistanceUnit.INCHES), angleUnit.convertTo(HALAngleUnit.RADIANS).apply(perpendicularWheelPose.getHeading())
        ), trackingWheelConfig);
    }

    /**
     * Set the parallel and perpendicular weights.
     *
     * @param parallelWeight      The desired parallel weight.
     * @param perpendicularWeight The desired perpendicular weight.
     * @return This localizer.
     */
    public TwoWheelLocalizer setWeights(double parallelWeight, double perpendicularWeight) {
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
    public TwoWheelLocalizer reverseEncoder(String encoder) {
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
    public TwoWheelLocalizer setEncoderDirection(String encoder, Encoder.Direction encoderDirection) {
        encoders.get(encoder).setDirection(encoderDirection);
        return this;
    }

    /**
     * Remaps the imu axes.
     *
     * @param axesOrder The order of the axes.
     * @param axesSigns The signs of the axes.
     * @return This localizer.
     */
    public TwoWheelLocalizer remapIMUAxes(AxesOrder axesOrder, AxesSigns axesSigns) {
        BNO055IMUUtil.remapAxes(imu, axesOrder, axesSigns);
        return this;
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Nullable
    @Override
    public Double getHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                trackingWheelConfig.encoderTicksToInches(encoders.get(PARALLEL_WHEEL).getCurrentPosition()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(PERPENDICULAR_WHEEL).getCurrentPosition()) * PERPENDICULAR_WEIGHT
        );
    }

    @Nullable
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                trackingWheelConfig.encoderTicksToInches(encoders.get(PARALLEL_WHEEL).getCorrectedVelocity()) * PARALLEL_WEIGHT,
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