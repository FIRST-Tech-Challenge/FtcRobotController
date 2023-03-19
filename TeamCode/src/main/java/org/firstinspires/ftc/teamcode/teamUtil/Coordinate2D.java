package org.firstinspires.ftc.teamcode.teamUtil;

public class Coordinate2D {

    /**
     * All distance measurements are in mm
     * */
    public double x;
    public double y;

    public Coordinate2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void vector2DUpdate(double distance, Angle heading){
        this.x += distance*Math.toDegrees(Math.cos(Math.toRadians(heading.convertToAbsolute().getValue())));
        this.y += distance*Math.toDegrees(Math.sin(Math.toRadians(heading.convertToAbsolute().getValue())));
    }
}
