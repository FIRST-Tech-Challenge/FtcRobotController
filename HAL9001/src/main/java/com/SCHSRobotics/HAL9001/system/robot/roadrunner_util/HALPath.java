package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.SCHSRobotics.HAL9001.util.math.geometry.Point2D;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;

import org.jetbrains.annotations.NotNull;

import java.util.List;

public class HALPath {

    //The roadrunner trajectory associated with this wrapper class.
    private final Path path;
    //The coordinate mode specifying how coordinates should be entered and returned.
    private final CoordinateMode coordinateMode;

    /**
     * The constructor for HALTrajectory.
     *
     * @param path           The roadrunner trajectory associated with this wrapper class.
     * @param coordinateMode The coordinate mode specifying how coordinates should be entered and returned.
     */
    public HALPath(Path path, CoordinateMode coordinateMode) {
        this.coordinateMode = coordinateMode;
        this.path = path;
    }

    /**
     * Gets the start pose of the trajectory.
     *
     * @return The start pose of the trajectory.
     */
    public Pose2d start() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.start());
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
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.end());
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

    public Pose2d get(double lengthAlongPathInches) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.get(lengthAlongPathInches));
    }

    public Pose2d get(double lengthAlongPath, HALDistanceUnit distanceUnit) {
        return get(HALDistanceUnit.convert(lengthAlongPath, distanceUnit, HALDistanceUnit.INCHES));
    }

    public Pose2d get(double lengthAlongPath, HALDistanceUnit distanceUnit, HALAngleUnit angleUnit) {
        Pose2d pose = get(lengthAlongPath, distanceUnit);
        return new Pose2d(
                HALDistanceUnit.convert(pose.getX(), HALDistanceUnit.INCHES, distanceUnit),
                HALDistanceUnit.convert(pose.getY(), HALDistanceUnit.INCHES, distanceUnit),
                HALAngleUnit.RADIANS.convertTo(angleUnit).apply(pose.getHeading())
        );
    }

    public Pose2d get(double lengthAlongPathInches, HALAngleUnit angleUnit) {
        return get(lengthAlongPathInches, HALDistanceUnit.INCHES, angleUnit);
    }

    public Pose2d deriv(double s, double t) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.deriv(s, t));
    }

    public Pose2d deriv(double s) {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.deriv(s));
    }

    public Pose2d endDeriv() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.endDeriv());
    }

    public Pose2d endSecondDeriv() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.endSecondDeriv());
    }

    public double fastProject(@NotNull Point2D queryPoint, double projectGuess) {
        Pose2d correctCoordinatePose = coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(queryPoint.getX(), queryPoint.getY()));
        return path.fastProject(new Vector2d(correctCoordinatePose.getX(), correctCoordinatePose.getY()), projectGuess);
    }

    public double project(@NotNull Point2D queryPoint, double ds) {
        Pose2d correctCoordinatePose = coordinateMode.convertTo(CoordinateMode.ROADRUNNER).apply(new Pose2d(queryPoint.getX(), queryPoint.getY()));
        return path.project(new Vector2d(correctCoordinatePose.getX(), correctCoordinatePose.getY()), ds);
    }

    public Pose2d startDeriv() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.startDeriv());
    }

    public Pose2d startSecondDeriv() {
        return CoordinateMode.ROADRUNNER.convertTo(coordinateMode).apply(path.startSecondDeriv());
    }

    public double length() {
        return path.length();
    }

    /**
     * Gets all the markers in the trajectory.
     *
     * @return A list of all the markers in the trajectory.
     */
    public List<PathSegment> getMarkers() {
        return path.getSegments();
    }


    /**
     * Converts this trajectory to a roadrunner trajectory.
     *
     * @return The roadrunner trajectory version of this HAL Trajectory.
     */
    public Path toRoadrunner() {
        return path;
    }
}
