package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

public class TrajectorySolution {
    private double angularVelocity;
    private double elevation;
    private double distance;
    private double bearing;
    private double xOffset;
    private double velocity;

    public TrajectorySolution(double angularVelocity, double elevation, double distance, double bearing, double xOffset, double velocity) {
        this.angularVelocity = angularVelocity;
        this.elevation = elevation;
        this.distance = distance;
        this.bearing = bearing;
        this.xOffset = xOffset;
        this.velocity = velocity;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getElevation() {
        return elevation;
    }

    public double getDistance() {return distance;}

    public double getBearing() {return bearing;}

    public double getxOffset() {return xOffset;}

    public double getVelocity() {return velocity;}
}
