package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.SCHSRobotics.HAL9001.util.functional_interfaces.Function;
import com.SCHSRobotics.HAL9001.util.math.geometry.Point2D;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * A wrapper class for the roadrunner trajectory builder class that allows you to work in HAL coordinates instead of roadrunner coordinates.
 * <p>
 * Creation Date: 1/10/21
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see TrajectoryBuilder
 * @see Trajectory
 * @see HALTrajectory
 * @since 1.1.1
 */
public final class HALTrajectoryBuilder {
    //The trajectory builder associated with this wrapper class.
    private final TrajectoryBuilder trajectoryBuilder;
    //The coordinate mode specifying how coordinates should be entered and returned.
    private final CoordinateMode coordinateMode;
    //The distance units for all entered pose x/y coordinates.
    private final HALDistanceUnit distanceUnit;
    //The angle units for all entered pose headings.
    private final HALAngleUnit angleUnit;

    /**
     * The constructor for HALTrajectoryBuilder.
     *
     * @param trajectoryBuilder The roadrunner trajectory associated with this wrapper class.
     * @param coordinateMode    The coordinate mode specifying how coordinates should be entered and returned.
     * @param distanceUnit      The distance units for all entered pose x/y coordinates.
     * @param angleUnit         The angle units for all entered pose headings.
     */
    public HALTrajectoryBuilder(TrajectoryBuilder trajectoryBuilder, CoordinateMode coordinateMode, HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        this.trajectoryBuilder = trajectoryBuilder;
        this.coordinateMode = coordinateMode;
        this.distanceUnit = distanceUnit;
        this.angleUnit = angleUnit;
    }

    /**
     * The constructor for HALTrajectoryBuilder.
     *
     * @param trajectoryBuilder The roadrunner trajectory associated with this wrapper class.
     * @param coordinateMode    The coordinate mode specifying how coordinates should be entered and returned.
     * @param distanceUnit      The distance units for all entered pose x/y coordinates.
     */
    public HALTrajectoryBuilder(TrajectoryBuilder trajectoryBuilder, CoordinateMode coordinateMode, HALDistanceUnit distanceUnit) {
        this(trajectoryBuilder, coordinateMode, distanceUnit, HALAngleUnit.RADIANS);
    }

    /**
     * The constructor for HALTrajectoryBuilder.
     *
     * @param trajectoryBuilder The roadrunner trajectory associated with this wrapper class.
     * @param coordinateMode    The coordinate mode specifying how coordinates should be entered and returned.
     */
    public HALTrajectoryBuilder(TrajectoryBuilder trajectoryBuilder, CoordinateMode coordinateMode) {
        this(trajectoryBuilder, coordinateMode, HALDistanceUnit.INCHES, HALAngleUnit.RADIANS);
    }

    /**
     * Makes a line to the given position.
     *
     * @param endPosition             The position to make a line to.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder lineTo(@NotNull final Point2D endPosition, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.lineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec(),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Makes a line to the given position.
     *
     * @param endPosition The position to make a line to.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder lineTo(@NotNull Point2D endPosition) {
        trajectoryBuilder.lineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec()
        );
        return this;
    }

    /**
     * Makes a line to the given position while maintaining a constant heading.
     *
     * @param endPosition             The position to make a line to.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder lineToConstantHeading(@NotNull final Point2D endPosition, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.lineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec(),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Makes a line to the given position while maintaining a constant heading.
     *
     * @param endPosition The position to make a line to.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder lineToConstantHeading(@NotNull Point2D endPosition) {
        trajectoryBuilder.lineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec()
        );
        return this;
    }

    /**
     * Makes a line to the given position while changing heading linearly.
     *
     * @param endPose                 The pose to end the line at.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder lineToLinearHeading(@NotNull final Pose2d endPose, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.lineToLinearHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                        HALDistanceUnit.convert(endPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(endPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endPose.getHeading())
                )),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Makes a line to the given position while changing heading linearly.
     *
     * @param endPose The pose to end the line at.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder lineToLinearHeading(@NotNull Pose2d endPose) {
        trajectoryBuilder.lineToLinearHeading(
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
     * @param endPose                 The pose to end the line at.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder lineToSplineHeading(@NotNull final Pose2d endPose, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.lineToSplineHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                        HALDistanceUnit.convert(endPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(endPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endPose.getHeading())
                )),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Makes a line to the given position while changing heading in a spline.
     *
     * @param endPose The pose to end the line at.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder lineToSplineHeading(@NotNull Pose2d endPose) {
        trajectoryBuilder.lineToSplineHeading(
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
     * @param endPosition             The position to strafe to.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder strafeTo(@NotNull final Point2D endPosition, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.strafeTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec(),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Strafes to the given position.
     *
     * @param endPosition The position to strafe to.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder strafeTo(@NotNull Point2D endPosition) {
        trajectoryBuilder.strafeTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec()
        );
        return this;
    }

    /**
     * Moves forward a given distance.
     *
     * @param distance                The distance to move forward.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder forward(final double distance, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.forward(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES), velConstraintOverride, accelConstraintOverride);
        return this;
    }

    /**
     * Moves forward a given distance.
     *
     * @param distance The distance to move forward.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder forward(double distance) {
        trajectoryBuilder.forward(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES));
        return this;
    }

    /**
     * Moves backward a given distance.
     *
     * @param distance                The distance to move backward.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder back(final double distance, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.back(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES), velConstraintOverride, accelConstraintOverride);
        return this;
    }

    /**
     * Moves backward a given distance.
     *
     * @param distance The distance to move backward.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder back(double distance) {
        trajectoryBuilder.back(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES));
        return this;
    }

    /**
     * Strafes left a given distance.
     *
     * @param distance                The distance to strafe left.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder strafeLeft(final double distance, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.strafeLeft(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES), velConstraintOverride, accelConstraintOverride);
        return this;
    }

    /**
     * Strafes left a given distance.
     *
     * @param distance The distance to strafe left.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder strafeLeft(double distance) {
        trajectoryBuilder.strafeLeft(distance);
        return this;
    }

    /**
     * Strafes right a given distance.
     *
     * @param distance                The distance to strafe right.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder strafeRight(final double distance, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.strafeRight(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES), velConstraintOverride, accelConstraintOverride);
        return this;
    }

    /**
     * Strafes right a given distance.
     *
     * @param distance The distance to strafe right.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder strafeRight(double distance) {
        trajectoryBuilder.strafeRight(HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES));
        return this;
    }

    /**
     * Splines to a given position.
     *
     * @param endPosition             The position to spline to.
     * @param endTangent              The ending heading.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder splineTo(@NotNull final Point2D endPosition, final double endTangent, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.splineTo(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec(),
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endTangent),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Splines to a given position.
     *
     * @param endPosition The position to spline to.
     * @param endTangent  The ending heading.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder splineTo(@NotNull Point2D endPosition, double endTangent) {
        trajectoryBuilder.splineTo(
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
     * @param endPosition             The position to spline to.
     * @param endTangent              The ending heading.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder splineToConstantHeading(@NotNull final Point2D endPosition, final double endTangent, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.splineToConstantHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(endPosition.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(endPosition.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec(),
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endTangent),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Splines to a given position while maintaining a constant heading.
     *
     * @param endPosition The position to spline to.
     * @param endTangent  The ending heading.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder splineToConstantHeading(@NotNull Point2D endPosition, double endTangent) {
        trajectoryBuilder.splineToConstantHeading(
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
     * @param endPose                 The pose to spline to.
     * @param endTangent              The ending path heading.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder splineToLinearHeading(@NotNull final Pose2d endPose, final double endTangent, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.splineToLinearHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                        HALDistanceUnit.convert(endPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(endPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endPose.getHeading())
                )),
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endTangent),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Splines to a given position while changing heading linearly.
     *
     * @param endPose    The pose to spline to.
     * @param endTangent The ending path heading.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder splineToLinearHeading(@NotNull Pose2d endPose, double endTangent) {
        trajectoryBuilder.splineToLinearHeading(
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
     * @param endPose                 The pose to spline to.
     * @param endTangent              The ending path heading.
     * @param velConstraintOverride   The velocity constraints on this motion.
     * @param accelConstraintOverride The acceleration constraints on this motion.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder splineToSplineHeading(@NotNull final Pose2d endPose, final double endTangent, @Nullable TrajectoryVelocityConstraint velConstraintOverride, @Nullable TrajectoryAccelerationConstraint accelConstraintOverride) {
        trajectoryBuilder.splineToSplineHeading(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(
                        HALDistanceUnit.convert(endPose.getX(), distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(endPose.getY(), distanceUnit, HALDistanceUnit.INCHES),
                        angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endPose.getHeading())
                )),
                angleUnit.convertTo(HALAngleUnit.RADIANS).apply(endTangent),
                velConstraintOverride,
                accelConstraintOverride
        );
        return this;
    }

    /**
     * Splines to a given position while changing heading in a spline.
     *
     * @param endPose    The pose to spline to.
     * @param endTangent The ending path heading.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder splineToSplineHeading(@NotNull Pose2d endPose, double endTangent) {
        trajectoryBuilder.splineToSplineHeading(
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
     * Adds a temporal marker to the trajectory.
     *
     * @param time     The time when the marker will run in seconds.
     * @param callback The code to run when the trajectory reaches this time value.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(double time, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addTemporalMarker(time, callback);
        return this;
    }

    /**
     * Adds a temporal marker to the trajectory.
     *
     * @param time     The time when the marker will run in seconds.
     * @param timeUnit The units of the time parameter.
     * @param callback The code to run when the trajectory reaches this time value.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(double time, HALTimeUnit timeUnit, MarkerCallback callback) {
        return addTemporalMarker(HALTimeUnit.convert(time, timeUnit, HALTimeUnit.SECONDS), callback);
    }

    /**
     * Adds a temporal marker to the trajectory.
     *
     * @param scale    A number that is multiplied by the trajectory duration.
     * @param offset   A time value in seconds that is added to scale*(trajectory duration).
     * @param callback The code to run when the trajectory reaches this time value.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(final double scale, final double offset, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addTemporalMarker(scale, offset, callback);
        return this;
    }

    /**
     * Adds a temporal marker to the trajectory.
     *
     * @param scale    A number that is multiplied by the trajectory duration.
     * @param offset   A time value in seconds that is added to scale*(trajectory duration).
     * @param timeUnit The units of the time parameter.
     * @param callback The code to run when the trajectory reaches this time value.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(final double scale, final double offset, HALTimeUnit timeUnit, @NotNull MarkerCallback callback) {
        return addTemporalMarker(scale, HALTimeUnit.convert(offset, timeUnit, HALTimeUnit.SECONDS), callback);
    }

    /**
     * Adds a temporal marker to the trajectory.
     *
     * @param time     A function that takes the duration of the trajectory in seconds as input and returns when the marker should be placed in seconds.
     * @param callback The code to run when the trajectory reaches this time value.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(@NotNull Function<Double, Double> time, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addTemporalMarker(time::apply, callback);
        return this;
    }

    /**
     * Adds a temporal marker to the trajectory.
     *
     * @param time     A function that takes the duration of the trajectory as input and returns when the marker should be placed.
     * @param timeUnit The units of the time parameter's input and output.
     * @param callback The code to run when the trajectory reaches this time value.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addTemporalMarker(@NotNull Function<Double, Double> time, HALTimeUnit timeUnit, @NotNull MarkerCallback callback) {
        return addTemporalMarker((Double duration) -> HALTimeUnit.convert(time.apply(HALTimeUnit.convert(duration, HALTimeUnit.SECONDS, timeUnit)), timeUnit, HALTimeUnit.SECONDS), callback);
    }

    /**
     * Adds a spatial marker to the trajectory.
     *
     * @param point    The point at which to add the marker.
     * @param callback The code to run when the trajectory reaches the closest point to this point value.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addSpatialMarker(@NotNull Point2D point, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addSpatialMarker(
                coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(
                        new Pose2d(HALDistanceUnit.convert(point.getX(), distanceUnit, HALDistanceUnit.INCHES), HALDistanceUnit.convert(point.getY(), distanceUnit, HALDistanceUnit.INCHES), 0)
                ).vec(),
                callback
        );
        return this;
    }

    /**
     * Adds a displacement marker to the trajectory.
     *
     * @param callback The code to run when the trajectory reaches this location along the path.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addDisplacementMarker(@NotNull MarkerCallback callback) {
        trajectoryBuilder.addDisplacementMarker(callback);
        return this;
    }

    /**
     * Adds a displacement marker to the trajectory.
     *
     * @param displacement The displacement along the path at which to activate the marker.
     * @param callback     The code to run when the trajectory reaches this location along the path.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addDisplacementMarker(double displacement, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addDisplacementMarker(HALDistanceUnit.convert(displacement, distanceUnit, HALDistanceUnit.INCHES), callback);
        return this;
    }

    /**
     * Adds a displacement marker to the trajectory.
     *
     * @param scale    A number that is multiplied by the trajectory path length.
     * @param offset   A distance value that is added to scale*(path length).
     * @param callback The code to run when the trajectory reaches this location along the path.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addDisplacementMarker(final double scale, final double offset, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addDisplacementMarker(scale, HALDistanceUnit.convert(offset, distanceUnit, HALDistanceUnit.INCHES), callback);
        return this;
    }

    /**
     * Adds a displacement marker to the trajectory.
     *
     * @param displacement A function that takes the path length of the trajectory as input and returns how far along the path the marker should be placed.
     * @param callback     The code to run when the trajectory reaches this location along the path.
     * @return This trajectory builder.
     */
    @NotNull
    public final HALTrajectoryBuilder addDisplacementMarker(@NotNull Function<Double, Double> displacement, @NotNull MarkerCallback callback) {
        trajectoryBuilder.addDisplacementMarker((Double pathLength) -> HALDistanceUnit.convert(displacement.apply(HALDistanceUnit.convert(pathLength, HALDistanceUnit.INCHES, distanceUnit)), distanceUnit, HALDistanceUnit.INCHES), callback);
        return this;
    }

    /**
     * Builds the trajectory.
     *
     * @return The HAL trajectory.
     */
    @NotNull
    public final HALTrajectory build() {
        return new HALTrajectory(trajectoryBuilder.build(), coordinateMode);
    }
}