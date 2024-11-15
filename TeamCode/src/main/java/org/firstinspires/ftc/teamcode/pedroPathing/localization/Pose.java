package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/**
 * This is the Pose class. It defines poses in 2D space, like the Pose2D class in Road Runner except
 * in the Pedro Pathing code so I don't have to import the Road Runner library. A Pose consists of
 * two coordinates defining a position and a third value for the heading, so basically just defining
 * any position and orientation the robot can be at, unless your robot can fly for whatever reason.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public class Pose {
    private double x;
    private double y;
    private double heading;

    /**
     * This creates a new Pose from a x, y, and heading inputs.
     *
     * @param setX the initial x value
     * @param setY the initial y value
     * @param setHeading the initial heading value
     */
    public Pose(double setX, double setY, double setHeading) {
        setX(setX);
        setY(setY);
        setHeading(setHeading);
    }

    /**
     * This creates a new Pose from x and y inputs. The heading is set to 0.
     *
     * @param setX the initial x value
     * @param setY the initial y value
     */
    public Pose(double setX, double setY) {
        this(setX, setY, 0);
    }

    /**
     * This creates a new Pose with no inputs and 0 for all values.
     */
    public Pose() {
        this(0,0,0);
    }

    /**
     * This sets the x value.
     *
     * @param set the x value
     */
    public void setX(double set) {
        x = set;
    }

    /**
     * This sets the y value.
     *
     * @param set the y value
     */
    public void setY(double set) {
        y = set;
    }

    /**
     * This sets the heading value.
     *
     * @param set the heading value
     */
    public void setHeading(double set) {
        heading = MathFunctions.normalizeAngle(set);
    }

    /**
     * This returns the x value.
     *
     * @return returns the x value
     */
    public double getX() {
        return x;
    }

    /**
     * This returns the y value.
     *
     * @return returns the y value
     */
    public double getY() {
        return y;
    }

    /**
     * This returns the heading value.
     *
     * @return returns the heading value
     */
    public double getHeading() {
        return heading;
    }

    /**
     * This returns the Pose as a Vector. Naturally, the heading data in the Pose cannot be included
     * in the Vector.
     *
     * @return returns the Pose as a Vector
     */
    public Vector getVector() {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(x, y);
        return returnVector;
    }

    /**
     * This returns a new Vector with magnitude 1 pointing in the direction of the heading.
     *
     * @return returns a unit Vector in the direction of the heading
     */
    public Vector getHeadingVector() {
        return new Vector(1, heading);
    }

    /**
     * This adds all the values of an input Pose to this Pose. The input Pose's data will not be
     * changed.
     *
     * @param pose the input Pose
     */
    public void add(Pose pose) {
        setX(x + pose.getX());
        setY(y + pose.getY());
        setHeading(heading + pose.getHeading());
    }

    /**
     * This subtracts all the values of an input Pose from this Pose. The input Pose's data will not
     * be changed.
     *
     * @param pose the input Pose
     */
    public void subtract(Pose pose) {
        setX(x - pose.getX());
        setY(y - pose.getY());
        setHeading(heading - pose.getHeading());
    }

    /**
     * This multiplies all the values of this Pose by a scalar.
     *
     * @param scalar the scalar
     */
    public void scalarMultiply(double scalar) {
        setX(x * scalar);
        setY(y * scalar);
        setHeading(heading * scalar);
    }

    /**
     * This divides all the values of this Pose by a scalar.
     *
     * @param scalar the scalar
     */
    public void scalarDivide(double scalar) {
        setX(x / scalar);
        setY(y / scalar);
        setHeading(heading / scalar);
    }

    /**
     * This flips the signs of all values in this Pose by multiplying them by -1. Heading values are
     * still normalized to be between 0 and 2 * pi in value.
     */
    public void flipSigns() {
        setX(-x);
        setY(-y);
        setHeading(-heading);
    }

    /**
     * This returns if a Pose is within a specified accuracy of this Pose in terms of x position,
     * y position, and heading.
     *
     * @param pose the input Pose to check
     * @param accuracy the specified accuracy necessary to return true
     * @return returns if the input Pose is within the specified accuracy of this Pose
     */
    public boolean roughlyEquals(Pose pose, double accuracy) {
        return MathFunctions.roughlyEquals(x, pose.getX(), accuracy) && MathFunctions.roughlyEquals(y, pose.getY(), accuracy) && MathFunctions.roughlyEquals(MathFunctions.getSmallestAngleDifference(heading, pose.getHeading()), 0, accuracy);
    }

    /**
     * This checks if the input Pose is within 0.0001 in all values to this Pose.
     *
     * @param pose the input Pose
     * @return returns if the input Pose is within 0.0001 of this Pose
     */
    public boolean roughlyEquals(Pose pose) {
        return roughlyEquals(pose, 0.0001);
    }

    /**
     * This creates a copy of this Pose that points to a new memory location.
     *
     * @return returns a deep copy of this Pose
     */
    public Pose copy() {
        return new Pose(getX(), getY(), getHeading());
    }
}
