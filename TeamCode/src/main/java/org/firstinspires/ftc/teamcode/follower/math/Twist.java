/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.math;

public class Twist {
    private static final Twist ZERO = new Twist(0, 0, 0);
    public final double vx;
    public final double vy;
    public final double omega;

    public Twist(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public static Twist zero() {
        return ZERO;
    }

    public static Twist fromVector(Vector2D vector) {
        return new Twist(vector.x(), vector.y(), 0);
    }

    public static Twist fromPose(Pose pose) {
        return new Twist(pose.x(), pose.y(), pose.heading());
    }

    public static Twist riemannianLog(Pose a, Pose b) {
        return a.invert().compose(b).log();
    }

    public Velocity toVelocity(double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new Velocity(vx * cos - vy * sin, vx * sin + vy * cos, omega);
    }

    public Vector toVector() {
        return new Vector(vx, vy, omega);
    }

    public Matrix toMatrix() {
        return new Matrix(new double[][] {
            {0.0, -omega, vx},
            {omega, 0.0, vy},
            {0.0, 0.0, 0.0}
        });
    }

    public Twist plus(Twist other) {
        return new Twist(vx + other.vx, vy + other.vy, omega + other.omega);
    }

    public Twist minus(Twist other) {
        return new Twist(vx - other.vx, vy - other.vy, omega - other.omega);
    }

    public Twist times(double scalar) {
        return new Twist(vx * scalar, vy * scalar, omega * scalar);
    }

    public Vector2D toVector2D() {
        return Vector2D.cartesian(vx, vy);
    }

    public double linSpeed() {
        return Math.sqrt(vx * vx + vy * vy);
    }

    @Override
    public String toString() {
        return "Twist{" + "vx=" + vx + ", vy=" + vy + ", omega=" + omega + '}';
    }
}
