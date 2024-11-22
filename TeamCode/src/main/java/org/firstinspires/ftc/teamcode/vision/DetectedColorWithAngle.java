package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Rect;
import org.opencv.core.Point;

public class DetectedColorWithAngle extends DetectedColor {
    private double angle; // Angle in degrees
    private double[] color; // Color represented as an array [Y, B, R]
    private Point center; // Center point of the bounding box

    // Constructor
    public DetectedColorWithAngle(String colorName, Rect boundingBox, double area, double angle, double[] color) {
        super(colorName, boundingBox, area);
        this.angle = angle;
        this.color = color;
        this.center = calculateCenter(boundingBox);
    }

    // Calculate the center of the bounding box
    private Point calculateCenter(Rect boundingBox) {
        double centerX = boundingBox.x + boundingBox.width / 2.0;
        double centerY = boundingBox.y + boundingBox.height / 2.0;
        return new Point(centerX, centerY);
    }

    // Getter for angle
    public double getAngle() {
        return angle;
    }

    // Getter for color
    public double[] getColor() {
        return color;
    }

    // Getter for center
    public Point getCenter() {
        return center;
    }


    @Override
    public String toString() {
        return "DetectedColorWithAngle{" +
                "colorName='" + colorName + '\'' +
                ", boundingBox=" + boundingBox +
                ", area=" + area +
                ", angle=" + angle +
                ", color=[Y=" + color[0] + ", B=" + color[1] + ", R=" + color[2] + "]" +
                ", center=(" + center.x + ", " + center.y + ")" +
                '}';
    }
}
