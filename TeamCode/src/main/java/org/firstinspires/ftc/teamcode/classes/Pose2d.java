package org.firstinspires.ftc.teamcode.classes;

public class Pose2d {
    public double x;
    public double y;
    public double theta;

    public Pose2d (double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public Vector2d headingVec() {
        return new Vector2d(Math.cos(theta), Math.sin(theta));
    }

    public Pose2d times(double scalar) {
        return new Pose2d(scalar * x, scalar * y, scalar * theta);
    }
}