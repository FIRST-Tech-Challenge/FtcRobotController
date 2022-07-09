package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class tracker {
    public static double xpos=0;
    public static double ypos=0;
    public static double angle=0;
    public enum TrackType {
        ENCODER,ODOMETRY,ODOMETRY_TOUCH,ODOMETRY_TOUCH_ULTRA
    }

    protected LinearOpMode op = null;

    public tracker(LinearOpMode opMode) {
    }
    abstract public void track();
    public void setPosition(double xPos, double yPos, double Angle) {
        xpos=xPos;
        ypos=yPos;
        angle=Angle;
    }
}