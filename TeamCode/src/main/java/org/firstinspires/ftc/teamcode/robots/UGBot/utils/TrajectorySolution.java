package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

public class TrajectorySolution {
    private double angularVelocity;
    private double elevation;
    private double distance;
    private double bearing;

    public TrajectorySolution(double angularVelocity, double elevation, double distance, double bearing) {
        this.angularVelocity = angularVelocity;
        this.elevation = elevation;
        this.distance = distance;
        this.bearing = bearing;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getElevation() {
        return elevation;
    }

    public double getDistance() {return distance;}

    public double getBearing() {return bearing;}
}
