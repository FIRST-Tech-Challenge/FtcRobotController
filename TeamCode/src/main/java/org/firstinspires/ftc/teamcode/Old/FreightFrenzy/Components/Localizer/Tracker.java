package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Localizer;


public abstract class Tracker {
    public static double xpos=0,ypos=0,angle=0,Velocity=0,xVelocity=0,yVelocity=0,aVelocity=0;
    public enum TrackType {
        ENCODER,ODOMETRY,ODOMETRY_TOUCH,ODOMETRY_TOUCH_ULTRA
    }

    public Tracker() {
    }
    abstract public void track();
    public double[] getPos() {
        return new double[] {xpos,ypos,angle};
    }
    public double[] getVelocity(){
        return new double[] {Velocity,xVelocity,yVelocity,aVelocity};
    }
    public void setPosition(double xPos, double yPos, double Angle) {
        xpos=xPos;
        ypos=yPos;
        angle=Angle;
    }
}