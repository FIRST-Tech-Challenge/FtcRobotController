package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

/**
 * A wrapper class for the roadrunner trajectory class that allows you to work in HAL coordinates instead of roadrunner coordinates.
 * <p>
 * Creation Date: 1/10/21
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see TrajectoryBuilder
 * @see Trajectory
 * @see HALTrajectoryBuilder
 * @since 1.1.1
 */
public class HALTrajectory {
    //The roadrunner trajectory associated with this wrapper class.
    private final Trajectory trajectory;
    //The coordinate mode specifying how coordinates should be entered and returned.
    private final CoordinateMode coordinateMode;

    /**
     * The constructor for HALTrajectory.
     *
     * @param trajectory     The roadrunner trajectory associated with this wrapper class.
     * @param coordinateMode The coordinate mode specifying how coordinates should be entered and returned.
     */
    public HALTrajectory(Trajectory trajectory, CoordinateMode coordinateMode) {
        this.coordinateMode = coordinateMode;
        this.trajectory = trajectory;
    }

    /**
     * Gets the velocity of the trajectory at a given time.
     *
     * @param timeSeconds The time value in seconds.
     * @return The robot target velocity at the given time value.
     */
    public Pose2d velocity(double timeSeconds) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.velocity(timeSeconds));
    }

    /**
     * Gets the velocity of the trajectory at a given time.
     *
     * @param time     The time value.
     * @param timeUnit The unit of the time parameter.
     * @return The robot target velocity at the given time value.
     */
    public Pose2d velocity(double time, HALTimeUnit timeUnit) {
        return velocity(HALTimeUnit.convert(time, timeUnit, HALTimeUnit.SECONDS));
    }

    /**
     * Gets the acceleration of the trajectory at a given time.
     *
     * @param timeSeconds The time value in seconds.
     * @return The robot target acceleration at the given time value.
     */
    public Pose2d acceleration(double timeSeconds) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.acceleration(timeSeconds));
    }

    /**
     * Gets the acceleration of the trajectory at a given time.
     *
     * @param time     The time value.
     * @param timeUnit The unit of the time parameter.
     * @return The robot target acceleration at the given time value.
     */
    public Pose2d acceleration(double time, HALTimeUnit timeUnit) {
        return acceleration(HALTimeUnit.convert(time, timeUnit, HALTimeUnit.SECONDS));
    }

    /**
     * Gets the trajectory estimated duration in seconds.
     *
     * @return The trajectory estimated duration in seconds.
     */
    public double duration() {
        return trajectory.duration();
    }

    /**
     * Gets the trajectory estimated duration.
     *
     * @param timeUnit The units for the duration.
     * @return The trajectory estimated duration.
     */
    public double duration(HALTimeUnit timeUnit) {
        return HALTimeUnit.convert(duration(), HALTimeUnit.SECONDS, timeUnit);
    }

    /**
     * Gets the start pose of the trajectory.
     *
     * @return The start pose of the trajectory.
     */
    public Pose2d start() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.start());
    }

    /**
     * Gets the start pose of the trajectory.
     *
     * @param distanceUnit The units of the pose x, y position.
     * @param angleUnit    The angle unit of the pose's heading.
     * @return The start pose of the trajectory.
     */
    public Pose2d start(HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        Pose2d startPose = start();
        return new Pose2d(
                HALDistanceUnit.convert(startPose.getX(), HALDistanceUnit.INCHES, distanceUnit),
                HALDistanceUnit.convert(startPose.getY(), HALDistanceUnit.INCHES, distanceUnit),
                HALAngleUnit.RADIANS.convertTo(angleUnit).apply(startPose.getHeading())
        );
    }

    /**
     * Gets the end pose of the trajectory.
     *
     * @return The end pose of the trajectory.
     */
    public Pose2d end() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.end());
    }

    /**
     * Gets the end pose of the trajectory.
     *
     * @param distanceUnit The units of the pose x/y position.
     * @param angleUnit    The angle unit of the pose's heading.
     * @return The end pose of the trajectory.
     */
    public Pose2d end(HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        Pose2d endPose = end();
        return new Pose2d(
                HALDistanceUnit.convert(endPose.getX(), HALDistanceUnit.INCHES, distanceUnit),
                HALDistanceUnit.convert(endPose.getY(), HALDistanceUnit.INCHES, distanceUnit),
                HALAngleUnit.RADIANS.convertTo(angleUnit).apply(endPose.getHeading())
        );
    }

    /**
     * Gets the robot target pose at a given time in the trajectory.
     *
     * @param timeSeconds The time value in seconds.
     * @return The robot target position at a given time in the trajectory.
     */
    public Pose2d get(double timeSeconds) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(trajectory.get(timeSeconds));
    }

    /**
     * Gets the robot target pose at a given time in the trajectory.
     *
     * @param time     The time value.
     * @param timeUnit The unit of the time parameter.
     * @return The robot target position at a given time in the trajectory.
     */
    public Pose2d get(double time, HALTimeUnit timeUnit) {
        return get(HALTimeUnit.convert(time, timeUnit, HALTimeUnit.SECONDS));
    }

    /**
     * Gets the robot target pose at a given time in the trajectory.
     *
     * @param timeSeconds  The time value in seconds.
     * @param distanceUnit The units of the pose x/y position.
     * @param angleUnit    The angle unit of the pose's heading.
     * @return The robot target position at a given time in the trajectory.
     */
    public Pose2d get(double timeSeconds, HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        Pose2d pose = get(timeSeconds);
        return new Pose2d(
                HALDistanceUnit.convert(pose.getX(), HALDistanceUnit.INCHES, distanceUnit),
                HALDistanceUnit.convert(pose.getY(), HALDistanceUnit.INCHES, distanceUnit),
                HALAngleUnit.RADIANS.convertTo(angleUnit).apply(pose.getHeading())
        );
    }

    /**
     * Gets the robot target pose at a given time in the trajectory.
     *
     * @param time         The time value.
     * @param timeUnit     The unit of the time parameter.
     * @param distanceUnit The units of the pose x/y position.
     * @param angleUnit    The angle unit of the pose's heading.
     * @return The robot target position at a given time in the trajectory.
     */
    public Pose2d get(double time, HALTimeUnit timeUnit, HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        return get(HALTimeUnit.convert(time, timeUnit, HALTimeUnit.SECONDS), distanceUnit, angleUnit);
    }

    /**
     * Gets all the markers in the trajectory.
     *
     * @return A list of all the markers in the trajectory.
     */
    public List<TrajectoryMarker> getMarkers() {
        return trajectory.getMarkers();
    }

    /**
     * Gets the path object associated with this trajectory.
     *
     * @return The path object associated with this trajectory.
     */
    public HALPath getPath() {
        return new HALPath(trajectory.getPath(), coordinateMode);
    }

    /**
     * Gets the motion profile associated with this trajectory.
     *
     * @return The motion profile associated with this trajectory.
     */
    public MotionProfile getProfile() {
        return trajectory.getProfile();
    }

    /**
     * Converts this trajectory to a roadrunner trajectory.
     *
     * @return The roadrunner trajectory version of this HAL Trajectory.
     */
    public Trajectory toRoadrunner() {
        return trajectory;
    }
}
