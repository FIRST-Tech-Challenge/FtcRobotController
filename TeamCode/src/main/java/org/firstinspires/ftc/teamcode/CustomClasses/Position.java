package org.firstinspires.ftc.teamcode.CustomClasses;


public class Position{

    public double x;
    public double y;
    public double angle;

    public Position(){
        x = 0.0;
        y = 0.0;
        angle = 0.0;
    }

    public Position(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public void set(Position pose) {
        this.x = pose.x;
        this.y = pose.y;
        this.angle = pose.angle;
    }
}
