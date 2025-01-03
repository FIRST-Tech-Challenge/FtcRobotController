package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;


import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;

public class SampleDetection {
    public ColorRange color;
    public double width;
    public double height;
    public double angle;
    public double x;
    public double y;
    public enum SampleRotation {
        HORIZONTAL,
        VERTICAL
    }
    public SampleRotation sampleRotation;
    public RotatedRect boxFit;
    public boolean exists = true;
    public SampleDetection (ColorRange colorRange, RotatedRect rotatedRect, boolean exists){
        this.color = colorRange;
        this.width = rotatedRect.size.width;
        this.height = rotatedRect.size.height;
        this.angle = rotatedRect.angle;
        this.x = rotatedRect.center.x;
        this.y = rotatedRect.center.y;
        this.exists = exists;
        this.boxFit = rotatedRect;
        if (this.width > this.height){
            this.sampleRotation = SampleRotation.HORIZONTAL;
        }
        else {
            this.sampleRotation = SampleRotation.VERTICAL;
        }
    }
}
