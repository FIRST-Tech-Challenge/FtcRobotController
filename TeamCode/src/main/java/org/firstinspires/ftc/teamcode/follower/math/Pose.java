/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.math;

import org.firstinspires.ftc.teamcode.follower.utils.Angle;

public class Pose {
    private static final Pose ZERO = new Pose(0, 0, 0);
    private final double x;
    private final double y;
    private final double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = Angle.normalize(heading);
    }

    public Pose(double x, double y) {
        this(x, y, 0);
    }

    public static Pose zero() {
        return ZERO;
    }

    public static Pose interpolate(Pose lowerPose, Pose upperPose, double ratio) {
        double x = lowerPose.x() + ratio * (upperPose.x() - lowerPose.x());
        double y = lowerPose.y() + ratio * (upperPose.y() - lowerPose.y());
        double headingDiff = Angle.normalizeSigned(upperPose.heading() - lowerPose.heading());
        double heading = Angle.normalize(lowerPose.heading() + ratio * headingDiff);
        return new Pose(x, y, heading);
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double heading() {
        return heading;
    }

    public Pose withX(double x) {
        return new Pose(x, y, heading);
    }

    public Pose withY(double y) {
        return new Pose(x, y, heading);
    }

    public Pose withHeading(double heading) {
        return new Pose(x, y, heading);
    }

    public Vector2D toVector2D() {
        return Vector2D.cartesian(x, y);
    }

    public Matrix toMatrix() {
        return Matrix.createTransformation(x, y, heading);
    }

    public Pose exp(Velocity velocity, double time) {
        return new Pose(x + velocity.vx * time, y + velocity.vy * time, heading + velocity.omega * time);
    }

    public Pose exp(Velocity velocity) {
        return exp(velocity, 1.0);
    }

    public Pose exp(Twist twist, double time) {
        if (Math.abs(twist.omega) < 1e-9) return exp(twist.toVelocity(heading), time);
        double theta = twist.omega * time;
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        Vector2D localDeltas = Vector2D.cartesian(
                (sin * twist.vx - (1 - cos) * twist.vy) / twist.omega,
                ((1 - cos) * twist.vx + sin * twist.vy) / twist.omega);
        Vector2D globalDeltas = localDeltas.rotate(heading);
        return new Pose(x + globalDeltas.x(), y + globalDeltas.y(), heading + theta);
    }

    public Pose exp(Twist twist) {
        return exp(twist, 1.0);
    }

    public Pose compose(Pose other) {
        Vector2D translationDeltas = other.toVector2D().rotate(heading);
        return new Pose(x + translationDeltas.x(), y + translationDeltas.y(), heading + other.heading);
    }

    public Pose plus(Pose other) {
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose minus(Pose other) {
        return new Pose(x - other.x, y - other.y, heading - other.heading);
    }

    public Pose times(double scalar) {
        return new Pose(x * scalar, y * scalar, heading * scalar);
    }

    public Pose div(double scalar) {
        return new Pose(x / scalar, y / scalar, heading / scalar);
    }

    public double distance(Pose other) {
        return Math.hypot(x - other.x, y - other.y);
    }

    public Pose invert() {
        Vector2D invTrans = Vector2D.cartesian(-x, -y).rotate(-heading);
        return new Pose(invTrans.x(), invTrans.y(), -heading);
    }

    public Twist log() {
        double h = Angle.normalizeSigned(heading);
        double eps = 1e-6;
        if (Math.abs(h) < eps) {
            // Small-angle: Jacobian ≈ I
            return new Twist(x, y, h);
        }

        double A = Math.sin(h) / h;
        double B = (1.0 - Math.cos(h)) / h;
        double denom = A * A + B * B;
        if (denom < 1e-12) {
            return new Twist(x, y, h);
        }

        double vx = (A * x + B * y) / denom;
        double vy = (-B * x + A * y) / denom;

        return new Twist(vx, vy, h);
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + heading + ")";
    }
}
