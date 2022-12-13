package org.firstinspires.ftc.teamcode.pipelines;

public class DetectedCircle {
    public double x, y, radius;

    public DetectedCircle(){
        this(0, 0, 0);
    }

    public DetectedCircle(double x, double y, double radius){
        this.x = x;
        this.y = y;
        this.radius = radius;
    }
}
