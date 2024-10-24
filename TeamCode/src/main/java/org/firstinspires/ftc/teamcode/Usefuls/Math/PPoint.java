package org.firstinspires.ftc.teamcode.Usefuls.Math;

public class PPoint {
    private final double x,y,heading;
    private final double tolerance;
    public static double defaultTolerance = 0.5;

    public PPoint(double x, double y, double heading, double tolerance){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.tolerance = tolerance;
    }

    public PPoint(double x, double y, double heading){
        this(x, y, heading,defaultTolerance);
    }

    public PPoint(double x, double y){
        this(x, y, 0,defaultTolerance);
    }

    public PPoint(){
        this(0,0,0,defaultTolerance);
    }

    public double getX(){
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getTolerance(){
        return tolerance;
    }

    public double getDistance(PPoint other){
        return Math.sqrt((other.getX() - x)*(other.getX() - x) + (other.getY() - y)*(other.getY() - y));
    }

    public boolean isReached(PPoint other){
        return getDistance(other) <= tolerance;
    }
}