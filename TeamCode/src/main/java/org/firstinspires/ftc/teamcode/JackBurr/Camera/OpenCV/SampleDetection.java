package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;


import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;

public class SampleDetection {
    public ColorRange color;
    public double width;
    public double height;
    public double angle;
    public SampleDetection (ColorRange colorRange, RotatedRect rotatedRect){
        this.color = colorRange;
        this.width = rotatedRect.size.width;
        this.height = rotatedRect.size.height;
        this.angle = rotatedRect.angle;

    }
}
