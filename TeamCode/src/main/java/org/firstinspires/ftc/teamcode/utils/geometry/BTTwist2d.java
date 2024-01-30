package org.firstinspires.ftc.teamcode.utils.geometry;

/**
 * A change in distance along arc since the last pose update. We can use ideas
 * from differential calculus to create new Pose2ds from a Twist2d and vise
 * versa.
 *
 * <p>A Twist can be used to represent a difference between two poses.
 */
public class BTTwist2d {
    /**
     * Linear "dx" component.
     */
    public double dx;

    /**
     * Linear "dy" component.
     */
    public double dy;

    /**
     * Angular "dtheta" component (radians).
     */
    public double dtheta;

    public BTTwist2d() {
    }

    /**
     * Constructs a Twist2d with the given values.
     *
     * @param dx     Change in x direction relative to robot.
     * @param dy     Change in y direction relative to robot.
     * @param dtheta Change in angle relative to robot.
     */
    public BTTwist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    @Override
    public String toString() {
        return String.format("Twist2d(dX: %.2f, dY: %.2f, dTheta: %.2f)", dx, dy, dtheta);
    }

    /**
     * Checks equality between this Twist2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof BTTwist2d) {
            return Math.abs(((BTTwist2d) obj).dx - dx) < 1E-9
                    && Math.abs(((BTTwist2d) obj).dy - dy) < 1E-9
                    && Math.abs(((BTTwist2d) obj).dtheta - dtheta) < 1E-9;
        }
        return false;
    }
}