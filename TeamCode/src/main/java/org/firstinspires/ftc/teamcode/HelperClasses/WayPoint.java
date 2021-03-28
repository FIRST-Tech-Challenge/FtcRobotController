package org.firstinspires.ftc.teamcode.HelperClasses;

public class WayPoint {
    public WayPoint(double X, double Y, double angle, double speed){
        this.x = X;
        this.y = Y;
        this.angle = angle;
        this.speed = speed;
    }
    public void setWayPoint(double X, double Y, double angle) {
        this.x = X;
        this.y = Y;
        this.angle = angle;
    }

    public double x;
    public double y;
    public double angle;
    public double speed;
}
