package org.firstinspires.ftc.teamcode.commandBased.classes;

public class Vector2d {

    private double x;
    private double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d add(Vector2d vector2d) {
        double xSum = this.x + vector2d.getX();
        double ySum = this.y + vector2d.getY();
        return new Vector2d(xSum, ySum);
    }

    public Vector2d subtract(Vector2d vector2d) {
        double xDifference = this.x - vector2d.getX();
        double yDifference = this.y - vector2d.getY();
        return new Vector2d(xDifference, yDifference);
    }

    public double dotProduct(Vector2d vector2d) {
        double magnitudeProduct = getMagnitude() * vector2d.getMagnitude();
        double vectorsCos = Math.cos(Math.abs(vector2d.getAngle() - getAngle()));
        return magnitudeProduct * vectorsCos;
    }

    public Vector2d rotateBy(double angle) {
        return new Vector2d(Math.cos(angle)*x - Math.sin(angle)*y, Math.sin(angle)*x + Math.cos(angle)*y);
    }

    public Vector2d normalize() {
        return scale(1/getMagnitude());
    }

    public Vector2d scale(double scaleFactor) {
        return new Vector2d(x * scaleFactor, y * scaleFactor);
    }

    public double getMagnitude() {
        return Math.sqrt((Math.pow(x, 2)) + (Math.pow(y, 2)));
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
