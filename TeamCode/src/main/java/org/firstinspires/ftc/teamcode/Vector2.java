package org.firstinspires.ftc.teamcode;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public boolean isUnitVector() {
        return Math.abs(1 - getMagnitude()) < 1e-5;
    }

    public double getMagnitude() {
        return Math.sqrt((this.x * this.x) + (this.y * this.y));
    }
    public Vector2 normalize() {
        double magnitude = getMagnitude();
        if (magnitude == 0) {
            return new Vector2(0, 0);
        }
        return new Vector2(this.x / magnitude, this.y / magnitude);
    }
}
