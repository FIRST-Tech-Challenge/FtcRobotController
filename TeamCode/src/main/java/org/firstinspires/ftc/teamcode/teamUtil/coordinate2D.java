package org.firstinspires.ftc.teamcode.teamUtil;

public class coordinate2D {

    /**
     * All distance measurements are in mm
     * */
    public double x;
    public double y;

    public coordinate2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void vector2DUpdate(double distance, angle heading){
        this.x += distance*Math.toDegrees(Math.cos(Math.toRadians(heading.convertToAbsolute().value)));
        this.y += distance*Math.toDegrees(Math.sin(Math.toRadians(heading.convertToAbsolute().value)));
    }
}
