/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.math;

public class Vector2D {
    private static final Vector2D ZERO = new Vector2D(0, 0);
    private static final Vector2D I_HAT = new Vector2D(1, 0);
    private static final Vector2D J_HAT = new Vector2D(0, 1);
    private final double x;
    private final double y;

    private Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2D cartesian(double x, double y) {
        return new Vector2D(x, y);
    }

    public static Vector2D polar(double magnitude, double theta) {
        return new Vector2D(magnitude * Math.cos(theta), magnitude * Math.sin(theta));
    }

    public static Vector2D unit(double angle) {
        return polar(1, angle);
    }

    public static Vector2D zero() {
        return ZERO;
    }

    public static Vector2D iHat() {
        return I_HAT;
    }

    public static Vector2D jHat() {
        return J_HAT;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public double magnitudeSquared() {
        return x * x + y * y;
    }

    public Vector2D normalized() {
        double magnitude = magnitude();
        if (magnitude < 1e-9) throw new IllegalArgumentException("Cannot normalize 0 vector");
        return div(magnitude);
    }

    public Vector2D plus(Vector2D other) {
        return new Vector2D(x + other.x, y + other.y);
    }

    public Vector2D minus(Vector2D other) {
        return new Vector2D(x - other.x, y - other.y);
    }

    public Vector2D times(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    public Vector2D div(double scalar) {
        return new Vector2D(x / scalar, y / scalar);
    }

    public Vector2D rotate(double angle) {
        return new Vector2D(x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle));
    }

    public double dot(Vector2D other) {
        return x * other.x + y * other.y;
    }

    public double det(Vector2D other) {
        return x * other.y - y * other.x;
    }

    public Vector2D projectOnto(Vector2D other) {
        return other.times(dot(other) / other.dot(other));
    }

    public double theta() {
        if (isZero()) return 0;
        return Math.atan2(y, x);
    }

    public Vector toVector() {
        return new Vector(x, y);
    }

    public Vector2D transform(Matrix m) {
        return this.toVector().transform(m).toVector2D();
    }

    public boolean isZero() {
        return magnitudeSquared() < 1e-9;
    }

    public double quadraticForm(Matrix m) {
        return dot(transform(m));
    }

    public double angleTo(Vector2D other) {
        if (this.isZero() || other.isZero()) {
            throw new IllegalArgumentException("Cannot calculate angle to or from a zero vector.");
        }
        double cosTheta = dot(other) / (magnitude() * other.magnitude());
        cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));
        return Math.acos(cosTheta);
    }

    public double distance(Vector2D other) {
        return Math.hypot(x - other.x, y - other.y);
    }

    public double distanceSquared(Vector2D other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return dx * dx + dy * dy;
    }

    public Vector2D hadamardProduct(Vector2D other) {
        return new Vector2D(this.x * other.x, this.y * other.y);
    }

    /**
     * this divided by other
     */
    public Vector2D elementDivision(Vector2D other) {
        return new Vector2D(this.x / other.x, this.y / other.y);
    }

    public Vector2D perpendicularLeft() {
        return new Vector2D(-y, x);
    }

    public Pose toPose(double heading) {
        return new Pose(x, y, heading);
    }

    public Pose toPose() {
        return new Pose(x, y, 0);
    }

    public Vector2D abs() {
        return Vector2D.cartesian(Math.abs(x), Math.abs(y));
    }

    public Vector2D toBodyFrame(double heading) {
        return rotate(-heading);
    }

    public Vector2D toWorldFrame(double heading) {
        return rotate(heading);
    }

    public Velocity toVelocity(double angularVelocity) {
        return new Velocity(x, y, angularVelocity);
    }

    public Velocity toVelocity() {
        return new Velocity(x, y, 0);
    }

    public Twist toTwist(double angularVelocity) {
        return new Twist(x, y, angularVelocity);
    }

    public Twist toTwist() {
        return new Twist(x, y, 0);
    }

    @Override
    public String toString() {
        return "Vector2D{" + "x=" + x + ", y=" + y + '}';
    }
}
