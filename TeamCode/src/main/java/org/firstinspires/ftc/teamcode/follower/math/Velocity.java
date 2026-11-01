/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.math;

public class Velocity {
    public final double vx;
    public final double vy;
    public final double omega;
    private static final Velocity ZERO = new Velocity(0, 0, 0);

    public Velocity(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public Twist toTwist(double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new Twist(vx * cos + vy * sin, -vx * sin + vy * cos, omega);
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

    public Velocity plus(Velocity other) {
        return new Velocity(vx + other.vx, vy + other.vy, omega + other.omega);
    }

    public Velocity minus(Velocity other) {
        return new Velocity(vx - other.vx, vy - other.vy, omega - other.omega);
    }

    public Velocity scale(double time) {
        return new Velocity(vx * time, vy * time, omega * time);
    }

    public Vector2D toVector2D() {
        return Vector2D.cartesian(vx, vy);
    }

    public static Velocity zero() {
        return ZERO;
    }

    public static Velocity fromVector(Vector2D vector) {
        return new Velocity(vector.x(), vector.y(), 0);
    }

    public static Velocity fromPose(Pose pose) {
        return new Velocity(pose.x(), pose.y(), pose.heading());
    }

    public double linSpeed() {
        return Math.sqrt(vx * vx + vy * vy);
    }

    @Override
    public String toString() {
        return "Velocity{" + "vx=" + vx + ", vy=" + vy + ", omega=" + omega + '}';
    }
}
