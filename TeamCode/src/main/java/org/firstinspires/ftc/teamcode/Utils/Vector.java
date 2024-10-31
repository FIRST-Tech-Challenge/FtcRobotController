package org.firstinspires.ftc.teamcode.Utils;

public final class Vector {

    public double x;
    public double y;

    public Vector(final double x, final double y) {
        this.x = x;
        this.y = y;
    }

    public double getDistanceInCm(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static Vector zero() {
        return new Vector(0, 0);
    } //returns Vector zero

    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    public double getAngle() {
        if (x == 0 && y == 0) {
            return 0;
        } else {
            return Math.atan2(y, x);
        }
    }

    public Vector scale(final double scalingFactor) {
        return new Vector(x * scalingFactor, y * scalingFactor);
    }

    public Vector unit() {
        if (x == 0 && y == 0) {
            return this;
        } else {
            return scale(1 / norm());
        }
    }
    // returns maagal hyehida
    public static Vector unit(final double angle) {
        return new Vector(Math.cos(angle), Math.sin(angle));
    }

    public static Vector fromAngleAndRadius(final double theta, final double radius) {
        final double vectorX = Math.cos(theta) * radius;
        final double vectorY = Math.sin(theta) * radius;
        return new Vector(vectorX, vectorY);
    }

    public double dotProduct(final Vector other) { return x * other.x + y * other.y;}

    public Vector rotate(final double theta) {
        final double sinTheta = Math.sin(theta);
        final double cosTheta = Math.cos(theta);

        final double newX = x * cosTheta - y * sinTheta;
        final double newY = x * sinTheta + y * cosTheta;

        return new Vector(newX, newY);
    }

    public Vector rotate90(final boolean rotateCounterClockwise) {
        if (!rotateCounterClockwise) {
            return new Vector(-y, x);
        } else {
            return new Vector(y, -x);
        }
    }

    public Vector abs() {
        return new Vector(Math.abs(x), Math.abs(y));
    }

    public Vector add(final Vector other) {
        return new Vector(x + other.x, y + other.y);
    }

    public Vector subtract(final Vector other) {
        return new Vector(x - other.x, y - other.y);
    }

    public double project(final double angle) {
        final Vector unitVector = unit(angle);
        return dotProduct(unitVector);
    }

    public double project(final Vector other) {
        final double otherNorm = other.norm();
        return otherNorm != 0 ? dotProduct(other) / otherNorm : this.norm();
    }

    public static Vector longest(final Vector a, final Vector b) {
        return a.norm() > b.norm() ? a : b;
    }

    public static Vector shortest(final Vector a, final Vector b) {
        return a.norm() < b.norm() ? a : b;
    }

    public static Vector max(final Vector a, final Vector b) {
        return new Vector(Math.max(a.x, b.x), Math.max(a.y, b.y));
    }

    public static Vector min(final Vector a, final Vector b) {
        return new Vector(Math.min(a.x, b.x), Math.min(a.y, b.y));
    }

    public static Vector sameXY(final double value) {
        return new Vector(value, value);
    }

    @Override
    public String toString() {
        return "x: " + x + " y: " + y;
    }

    public boolean equals(final Vector other) {
        return x == other.x && y == other.y;
    }

    public static double angleDifference(final Vector a, final Vector b) {
        final double normA = a.norm();
        final double normB = b.norm();

        if (normA == 0 || normB == 0)
            return 0;

        return Math.acos(a.dotProduct(b) / (normA * normB));
    }
}