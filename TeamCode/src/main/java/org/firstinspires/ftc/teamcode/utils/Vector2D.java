package org.firstinspires.ftc.teamcode.utils;

public class Vector2D {
    public double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D toPolar() {
        return new Vector2D(Math.hypot(x, y), Math.atan2(x, y));
    }

    public Vector2D toCartesian() {
        return new Vector2D(x * Math.cos(y), x * Math.sin(y));
    }

    public Vector2D scale(double scalar) {
        x *= scalar;
        y *= scalar;
        return this;
    }

    public Vector2D scaleExp(double exp) {
        x = Math.pow(x, exp);
        y = Math.pow(y, exp);
        return this;
    }

    public Vector2D copy() {
        return new Vector2D(this.x, this.y);
    }
}
