package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.SCHSRobotics.HAL9001.util.functional_interfaces.Function;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * An enum used for determining which coordinate mode (HAL or ROADRUNNER) is being used and for converting between the two.
 */
public enum CoordinateMode {
    ROADRUNNER((Pose2d pose) -> new Pose2d(-pose.getY(), pose.getX(), pose.getHeading())),
    HAL((Pose2d pose) -> new Pose2d(pose.getY(), -pose.getX(), pose.getHeading()));

    //The converter that allows you to convert from one coordinate mode to the other.
    private final Function<Pose2d, Pose2d> converter;

    /**
     * The constructor for CoordinateMode.
     *
     * @param converter The conversion function that changes one coordinate mode into the other coordinate mode.
     */
    CoordinateMode(Function<Pose2d, Pose2d> converter) {
        this.converter = converter;
    }

    /**
     * Converts from one coordinate mode to another coordinate mode.
     *
     * @param coordinateMode The coordinate mode to covnert to.
     * @return A function that will convert from THIS coordinate mode to the given coordinate mode.
     */
    public Function<Pose2d, Pose2d> convertTo(CoordinateMode coordinateMode) {
        switch (this) {
            default:
            case HAL:
                if (coordinateMode == HAL) return (Pose2d pose) -> pose;
                else return converter;
            case ROADRUNNER:
                if (coordinateMode == HAL) return converter;
                else return (Pose2d pose) -> pose;
        }
    }

    public Function<Vector2D, Vector2D> convertVectorTo(CoordinateMode coordinateMode) {
        Function<Vector2D, Vector2D> vectorConverter = (Vector2D vector) -> {
            Pose2d rawVectorPose = new Pose2d(vector.getX(), vector.getY());
            Pose2d vectorPose = converter.apply(rawVectorPose);
            return new Vector2D(vectorPose.getX(), vectorPose.getY());
        };

        switch (this) {
            default:
            case HAL:
                if (coordinateMode == HAL) return (Vector2D vector) -> vector;
                else return vectorConverter;
            case ROADRUNNER:
                if (coordinateMode == HAL) return vectorConverter;
                else return (Vector2D vector) -> vector;
        }
    }
}