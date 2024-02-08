package org.firstinspires.ftc.teamcode.autoutils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class OtherAutoUtils {

    public enum Quadrants {
        BLUE_TOP,
        BLUE_BOTTOM,
        RED_TOP,
        RED_BOTTOM
    }

    public enum DefinedLocations {
        BLUE_BOARD(50, 35),
        RED_BOARD(50, -35);

        final Vector2d LOCATION;

        DefinedLocations(final double x, final double y) {
            LOCATION = new Vector2d(x, y);
        }

        public Vector2d getLocation() {
            return LOCATION;
        }
    }

    final static double X_SPLIT = -10;
    final static double Y_SPLIT = 0;

    public static Quadrants getCurrentQuadrant(final Pose2d pose) {
        // Blue Quadrants - Y > 0
        // Red Quadrants - Y < 0
        // Bottom Quadrants - X < -10
        // Top Quadrants - X > -10

        final boolean IN_BOTTOM_QUADRANTS = pose.getX() <= X_SPLIT;
        final boolean IN_BLUE_QUADRANTS = pose.getY() >= Y_SPLIT;

        if (IN_BLUE_QUADRANTS) {
            return (IN_BOTTOM_QUADRANTS) ? Quadrants.BLUE_BOTTOM : Quadrants.BLUE_TOP;
        }

        return (IN_BOTTOM_QUADRANTS) ? Quadrants.RED_BOTTOM : Quadrants.RED_TOP;
    }

    public static boolean isInBottomQuadrant(final Pose2d pose) {
        switch (getCurrentQuadrant(pose)) {
            case RED_BOTTOM:
            case BLUE_BOTTOM:
                return true;
            case RED_TOP:
            case BLUE_TOP:
                return false;
        }

        return false;
    }

    public static boolean isInBlueQuadrant(final Pose2d pose) {
        switch (getCurrentQuadrant(pose)) {
            case BLUE_TOP:
            case BLUE_BOTTOM:
                return true;
            case RED_TOP:
            case RED_BOTTOM:
                return false;
        }

        return false;
    }

    /**
     * Checks whether x is in range of the midpoint
     * @param x the value to check if it's in range
     * @param range the range to check whether x falls under
     * @param midpoint the point to add the range to
     * @return whether x is in range of the midpoint
     */
    public static Boolean withinRange(final double x, final double range, final double midpoint) {
        final double midpoint_range_pos = midpoint + range;
        final double midpoint_range_neg = midpoint - range;

        return (!(x > midpoint_range_pos)) && (!(x < midpoint_range_neg));
    }

    public static Vector2d avoidTruss(final Pose2d startingPose) {
        final double X = startingPose.getX();
        final double Y = startingPose.getY();

        final double MIN_DISTANCE = 5;
        final Quadrants QUADRANT = getCurrentQuadrant(startingPose);

        double newX = startingPose.getX();
        //double newY;

        if (withinRange(X, MIN_DISTANCE, X_SPLIT)) {
            newX = startingPose.getX() +
                    ((isInBottomQuadrant(startingPose)) ? -MIN_DISTANCE : MIN_DISTANCE);
        }

        return new Vector2d(newX, startingPose.getY());
    }
}
