package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.SCHSRobotics.HAL9001.util.math.geometry.Point2D;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.PathBuilder;

import org.jetbrains.annotations.NotNull;

public class HALPathBuilder {
    //The path builder associated with this wrapper class.
    private final PathBuilder pathBuilder;
    //The coordinate mode specifying how coordinates should be entered and returned.
    private final CoordinateMode coordinateMode;
    //The distance units for all entered pose x/y coordinates.
    private final HALDistanceUnit distanceUnit;
    //The angle units for all entered pose headings.
    private final HALAngleUnit angleUnit;

    /**
     * The constructor for HALPathBuilder.
     *
     * @param pathBuilder    The roadrunner path associated with this wrapper class.
     * @param coordinateMode The coordinate mode specifying how coordinates should be entered and returned.
     * @param distanceUnit   The distance units for all entered pose x/y coordinates.
     * @param angleUnit      The angle units for all entered pose headings.
     */
    public HALPathBuilder(PathBuilder pathBuilder, CoordinateMode coordinateMode, HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        this.pathBuilder = pathBuilder;
        this.coordinateMode = coordinateMode;
        this.distanceUnit = distanceUnit;
        this.angleUnit = angleUnit;
    }

    /**
     * The constructor for HALPathBuilder.
     *
     * @param pathBuilder    The roadrunner path associated with this wrapper class.
     * @param coordinateMode The coordinate mode specifying how coordinates should be entered and returned.
     * @param distanceUnit   The distance units for all entered pose x/y coordinates.
     */
    public HALPathBuilder(PathBuilder pathBuilder, CoordinateMode coordinateMode, HALDistanceUnit distanceUnit) {
        this(pathBuilder, coordinateMode, distanceUnit, HALAngleUnit.RADIANS);
    }

    /**
     * The constructor for HALPathBuilder.
     *
     * @param pathBuilder    The roadrunner path associated with this wrapper class.
     * @param coordinateMode The coordinate mode specifying how coordinates should be entered and returned.
     */
    public HALPathBuilder(PathBuilder pathBuilder, CoordinateMode coordinateMode) {
        this(pathBuilder, coordinateMode, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS);
    }

    /**
     * Makes a line to the given position.
     *
     * @param endPosition The position to make a line to.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder lineTo(@NotNull Point2D endPosition) {
        pathBuilder.lineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec()
        );
        return this;
    }

    /**
     * Makes a line to the given position while maintaining a constant heading.
     *
     * @param endPosition The position to make a line to.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder lineToConstantHeading(@NotNull Point2D endPosition) {
        pathBuilder.lineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec()
        );
        return this;
    }

    /**
     * Makes a line to the given position while changing heading linearly.
     *
     * @param endPose The pose to end the line at.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder lineToLinearHeading(@NotNull Pose2d endPose) {
        pathBuilder.lineToLinearHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                        HALDistanceUnit.convert(endPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(endPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endPose.getHeading())
                ))
        );
        return this;
    }

    /**
     * Makes a line to the given position while changing heading in a spline.
     *
     * @param endPose The pose to end the line at.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder lineToSplineHeading(@NotNull Pose2d endPose) {
        pathBuilder.lineToSplineHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                        HALDistanceUnit.convert(endPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(endPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endPose.getHeading())
                ))
        );
        return this;
    }

    /**
     * Strafes to the given position.
     *
     * @param endPosition The position to strafe to.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder strafeTo(@NotNull Point2D endPosition) {
        pathBuilder.strafeTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec()
        );
        return this;
    }

    /**
     * Moves forward a given distance.
     *
     * @param distance The distance to move forward.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder forward(double distance) {
        pathBuilder.forward(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES));
        return this;
    }

    /**
     * Moves backward a given distance.
     *
     * @param distance The distance to move backward.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder back(double distance) {
        pathBuilder.back(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES));
        return this;
    }

    /**
     * Strafes left a given distance.
     *
     * @param distance The distance to strafe left.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder strafeLeft(double distance) {
        pathBuilder.strafeLeft(distance);
        return this;
    }

    /**
     * Strafes right a given distance.
     *
     * @param distance The distance to strafe right.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder strafeRight(double distance) {
        pathBuilder.strafeRight(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES));
        return this;
    }

    /**
     * Splines to a given position.
     *
     * @param endPosition The position to spline to.
     * @param endTangent  The ending heading.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder splineTo(@NotNull Point2D endPosition, double endTangent) {
        pathBuilder.splineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec(),
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endTangent)
        );
        return this;
    }

    /**
     * Splines to a given position while maintaining a constant heading.
     *
     * @param endPosition The position to spline to.
     * @param endTangent  The ending heading.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder splineToConstantHeading(@NotNull Point2D endPosition, double endTangent) {
        pathBuilder.splineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec(),
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endTangent)
        );
        return this;
    }

    /**
     * Splines to a given position while changing heading linearly.
     *
     * @param endPose    The pose to spline to.
     * @param endTangent The ending path heading.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder splineToLinearHeading(@NotNull Pose2d endPose, double endTangent) {
        pathBuilder.splineToLinearHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                        HALDistanceUnit.convert(endPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(endPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endPose.getHeading())
                )),
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endTangent)
        );
        return this;
    }

    /**
     * Splines to a given position while changing heading in a spline.
     *
     * @param endPose    The pose to spline to.
     * @param endTangent The ending path heading.
     * @return This path builder.
     */
    @NotNull
    public final HALPathBuilder splineToSplineHeading(@NotNull Pose2d endPose, double endTangent) {
        pathBuilder.splineToSplineHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                        HALDistanceUnit.convert(endPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(endPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endPose.getHeading())
                )),
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endTangent)
        );
        return this;
    }

    /**
     * Builds the path.
     *
     * @return The HAL path.
     */
    @NotNull
    public final HALPath build() {
        return new HALPath(pathBuilder.build(), coordinateMode);
    }
}
