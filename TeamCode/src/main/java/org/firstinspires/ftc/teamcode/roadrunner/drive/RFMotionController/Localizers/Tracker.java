package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers;


public abstract class Tracker {
    protected double xpos = 0, ypos = 0, angle = 0, Velocity = 0, xVelocity = 0, yVelocity = 0, aVelocity = 0;

    public Tracker() {

    }

    public enum TrackType {
        ODOMETRY, ODOMETRY_IMU_LEFT, ODOMETRY_IMU_RIGHT, ROADRUN_ODOMETRY, ROADRUN_IMU_LEFT, ROADRUN_IMU_RIGHT
    }

    abstract public void update();
}