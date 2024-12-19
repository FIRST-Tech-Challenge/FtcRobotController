package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

/**
 * This is the Point class. This class handles storing information about the location of points in
 * 2D space in both Cartesian, or rectangular, and polar coordinates. Additionally, this contains
 * the method to find the distance between two Points.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/11/2024
 */
public class Point {

    // IMPORTANT NOTE: theta is defined in radians.
    // These are the values of the coordinate defined by this Point, in both polar and
    // Cartesian systems.
    private double r;
    private double theta;
    private double x;
    private double y;

    // these are used for ease of changing/setting identification
    public static final int POLAR = 0;
    public static final int CARTESIAN = 1;


    /**
     * This creates a new Point with coordinate inputs and a specified coordinate system.
     *
     * @param rOrX Depending on the coordinate system specified, this is either the r or x value.
     *         In polar coordinates, the r value is the distance from the origin.
     *         In Cartesian coordinates, the x value is the distance left/right from the origin.
     * @param thetaOrY Depending on the coordinate system specified, this is either the theta or
     *         y value.
     *         In polar coordinates, the theta value is the angle from the positive x-axis.
     *         Increasing theta moves in the counter-clockwise direction.
     *         In Cartesian coordinates, the y value is the distance up/down from the origin.
     * @param identifier this specifies what coordinate system the coordinate inputs are in.
     */
    public Point(double rOrX, double thetaOrY, int identifier) {
        setCoordinates(rOrX, thetaOrY, identifier);
    }

    /**
     * This creates a new Point from a Pose.
     *
     * @param pose the Pose.
     */
    public Point(Pose pose) {
        setCoordinates(pose.getX(), pose.getY(), CARTESIAN);
    }

    /**
     * This creates a new Point from a X and Y value.
     *
     * @param setX the X value.
     * @param setY the Y value.
     */
    public Point(double setX, double setY) {
        setCoordinates(setX, setY, CARTESIAN);
    }

    /**
     * This sets the coordinates of the Point using the specified coordinate system.
     *
     * @param rOrX Depending on the coordinate system specified, this is either the r or x value.
     *         In polar coordinates, the r value is the distance from the origin.
     *         In Cartesian coordinates, the x value is the distance left/right from the origin.
     * @param thetaOrY Depending on the coordinate system specified, this is either the theta or
     *         y value.
     *         In polar coordinates, the theta value is the angle from the positive x-axis.
     *         Increasing theta moves in the counter-clockwise direction.
     *         In Cartesian coordinates, the y value is the distance up/down from the origin.
     * @param identifier this specifies what coordinate system to use when setting values.
     */
    public void setCoordinates(double rOrX, double thetaOrY, int identifier) {
        double[] setOtherCoordinates;
        switch (identifier) { // this detects which coordinate system to use
            // there is no POLAR case since that's covered by the default
            case CARTESIAN:
                x = rOrX;
                y = thetaOrY;
                setOtherCoordinates = cartesianToPolar(x, y);
                r = setOtherCoordinates[0];
                theta = setOtherCoordinates[1];
                break;
            default:
                if (rOrX < 0) {
                    r = -rOrX;
                    theta = MathFunctions.normalizeAngle(thetaOrY + Math.PI);
                } else {
                    r = rOrX;
                    theta = MathFunctions.normalizeAngle(thetaOrY);
                }
                setOtherCoordinates = polarToCartesian(r, theta);
                x = setOtherCoordinates[0];
                y = setOtherCoordinates[1];
                break;
        }
    }

    /**
     * Calculates the distance between this Point and some other specified Point.
     *
     * @param otherPoint the other specified Point.
     * @return returns the distance between the two Points.
     */
    public double distanceFrom(Point otherPoint) {
        return Math.sqrt(Math.pow(otherPoint.getX() - x, 2) + Math.pow(otherPoint.getY() - y, 2));
    }

    /**
     * This takes in an r and theta value and converts them to Cartesian coordinates.
     *
     * @param r this is the r value of the Point being converted.
     * @param theta this is the theta value of the Point being converted.
     * @return this returns the x and y values, in that order, in an Array of doubles.
     */
    public static double[] polarToCartesian(double r, double theta) {
        return new double[]{r * Math.cos(theta), r * Math.sin(theta)};
    }

    /**
     * This takes in an x and y value and converts them to polar coordinates.
     *
     * @param x this is the x value of the Point being converted.
     * @param y this is the y value of the Point being converted.
     * @return this returns the r and theta values, in that order, in an Array of doubles.
     */
    public static double[] cartesianToPolar(double x, double y) {
        if (x == 0) {
            if (y > 0) {
                return new double[]{Math.abs(y), Math.PI / 2};
            } else {
                return new double[]{Math.abs(y), (3 * Math.PI) / 2};
            }
        }
        double r = Math.sqrt(x * x + y * y);
        if (x < 0) return new double[]{r, Math.PI + Math.atan(y / x)};
        if (y > 0) {
            return new double[]{r, Math.atan(y / x)};
        } else {
            return new double[]{r, (2 * Math.PI) + Math.atan(y / x)};
        }
    }

    /**
     * Returns the r value of this Point. This is a polar coordinate value.
     *
     * @return returns the r value.
     */
    public double getR() {
        return r;
    }

    /**
     * Returns the theta value of this Point. This is a polar coordinate value.
     *
     * @return returns the theta value.
     */
    public double getTheta() {
        return theta;
    }

    /**
     * Returns the x value of this Point. This is a Cartesian coordinate value.
     *
     * @return returns the x value.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y value of this Point. This is a Cartesian coordinate value.
     *
     * @return returns the y value.
     */
    public double getY() {
        return y;
    }

    /**
     * This creates a new Point with the same information as this Point, just pointing to a different
     * memory location. In other words, a deep copy.
     *
     * @return returns a copy of this Point.
     */
    public Point copy() {
        return new Point(getX(), getY(), CARTESIAN);
    }

    @NonNull
    @Override
    public String toString() {
        return "(" + getX() + ", " + getY() + ")";
    }
}