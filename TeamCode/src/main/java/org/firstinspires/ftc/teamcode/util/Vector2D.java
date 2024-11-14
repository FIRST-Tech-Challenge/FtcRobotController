package org.firstinspires.ftc.teamcode.util;


public class Vector2D {
    public double x,y;

    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static Vector2D fromHeadingAndMagnitude(double h, double m){
        return new Vector2D(Math.cos(h) * m, Math.sin(h) * m);
    }

    public Vector2D mult(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    public Vector2D divide(double scalar) {
        return new Vector2D(x / scalar, y / scalar);
    }

    public Vector2D subt(Vector2D other) {
        return new Vector2D(x - other.x, y - other.y);
    }

    public double dot(Vector2D other) {
        return x * other.x + y * other.y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D unit() {
        return this.divide(magnitude());
    }

    public Vector2D rotate(double angle) {
        return new Vector2D(
                x * Math.cos(angle) - y * Math.sin(angle),
                x * Math.sin(angle) + y * Math.cos(angle));
    }

    public double cross(Vector2D other) {
        return x * other.y - y * other.x;
    }

    @Override
    public String toString() {
        return String.format("{%.2f, %.2f}", x, y);
    }
}