package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Rect;

public class DetectedColor {
    public String colorName;
    public Rect boundingBox;
    public double area; // This will represent the contour area, used to sort by proximity

    public DetectedColor(String colorName, Rect boundingBox, double area) {
        this.colorName = colorName;
        this.boundingBox = boundingBox;
        this.area = area;
    }
}
