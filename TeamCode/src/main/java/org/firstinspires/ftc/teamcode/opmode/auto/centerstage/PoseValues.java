package org.firstinspires.ftc.teamcode.opmode.auto.centerstage;

public class PoseValues {
    public double x, y, heading;

    public PoseValues(double x, double y, double headingDeg) {
        this.x = x;
        this.y = y;
        this.heading = Math.toRadians(headingDeg);
    }
}