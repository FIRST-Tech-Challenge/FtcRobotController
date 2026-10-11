package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

public class Pose2d implements Cloneable {

    private double x;
    private double y;
    private double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(double x, double y) {
        this(x, y, 0.0);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void add(Pose2d p1) {
        this.x += p1.x;
        this.y += p1.y;
        this.heading += p1.heading;
    }

    public boolean isNaN() {
        return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading);
    }

    public double getDistanceFromPoint(Pose2d newPoint) { // distance equation
        double dx = x - newPoint.x;
        double dy = y - newPoint.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getErrorInX(Pose2d newPoint) { // distance equation
        return Math.abs(x - newPoint.x);
    }

    public double getErrorInY(Pose2d newPoint) { // distance equation
        return Math.abs(y - newPoint.y);
    }

    public void clipAngle() {
        heading = clipAngle(heading);
    }

    public static double clipAngle(double angle) {
        double a = angle;
        while (Math.abs(a) > Math.PI) {
            a -= Math.PI * 2.0 * Math.signum(a);
        }
        return a;
    }

    // public Pose2d convertPose2D(Pose2D pose) {
    //     return new Pose2d(pose.getX(), pose.getY(), pose.getHeading());
    // }

    @Override
    public Pose2d clone() {
        return new Pose2d(x, y, heading);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("(%.3f, %.3f, %.3f", x, y, heading);
    }
}
