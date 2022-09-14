package org.firstinspires.ftc.teamcode.League1.Common;

public class Point {
    private double x, y;

    public Point(double x, double y){
        this.x  = x;
        this.y = y;
    }

    public double getDirection() {
        return Math.atan2(y,x);
    }

    public double magFromOrigin(){
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }
}