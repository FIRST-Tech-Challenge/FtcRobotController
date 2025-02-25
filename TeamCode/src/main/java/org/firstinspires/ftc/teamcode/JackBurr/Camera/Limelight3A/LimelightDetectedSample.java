package org.firstinspires.ftc.teamcode.JackBurr.Camera.Limelight3A;


public class LimelightDetectedSample {
    public Orientation orientation;
    public double angle;
    public enum Orientation {
        UPRIGHT,
        SIDEWAYS
    }
    public LimelightDetectedSample(double angle, double width, double height,  Orientation orientation){
        this.orientation = orientation;
        this.angle = angle;
    }
}
