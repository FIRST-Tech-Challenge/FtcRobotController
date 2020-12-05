package org.firstinspires.ftc.teamcode.lib.geometry;

import org.firstinspires.ftc.teamcode.lib.util.Util;

import java.text.DecimalFormat;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 * <p>
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
public class Twist2d {
    protected static final Twist2d kIdentity = new Twist2d(0.0, 0.0, 0.0);

    public static Twist2d identity() {
        return kIdentity;
    }

    public final double dx;
    public final double dy;
    public final double dtheta; // Radians!

    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public Twist2d scaled(double scale) {
        return new Twist2d(dx * scale, dy * scale, dtheta * scale);
    }

    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    public double curvature() {
        if (Math.abs(dtheta) < Util.getEpsilon() && norm() < Util.getEpsilon())
            return 0.0;
        return dtheta / norm();
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    }
}