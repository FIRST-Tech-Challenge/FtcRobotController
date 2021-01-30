package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

public class TrajectorySolution {
    private double angularVelocity;
    private double theta;

    public TrajectorySolution(double angularVelocity, double theta) {
        this.angularVelocity = angularVelocity;
        this.theta = theta;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public double getTheta() {
        return theta;
    }
}
