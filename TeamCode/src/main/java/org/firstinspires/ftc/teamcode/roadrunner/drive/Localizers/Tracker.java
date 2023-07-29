package org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers;


public abstract class Tracker {
    public static double xpos=0,ypos=0,angle=0,Velocity=0,xVelocity=0,yVelocity=0,aVelocity=0;
    public enum TrackType {
        ODOMETRY,ODOMETRY_IMU_LEFT, ODOMETRY_IMU_RIGHT
    }

    public Tracker(TrackType trackType) {

    }
    abstract public void update();
    public double[] getPos() {
        return new double[] {xpos,ypos,angle};
    }
    public double[] getVelocity(){
        return new double[] {Velocity,xVelocity,yVelocity,aVelocity};
    }
}