package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Rect;
import org.opencv.core.Point;

public class DetectedColorWithAngle extends DetectedColor {
    private double angle; // Angle in degrees
    private double[] color; // Color represented as an array [Y, B, R]
    private Point center; // Center point of the bounding box

    // Constructor
    public DetectedColorWithAngle(Rect boundingBox, double area, double angle, double[] color) {
        super(calculateColorName(color), boundingBox, area); // Infer colorName from the double[] color
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

    // Getter for colorName (from superclass DetectedColor)
    public String getColorName() {
        return this.colorName; // colorName is inherited from DetectedColor
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

    // Override toString for debugging
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

    // Method to calculate the color name based on the color array [Y, B, R]
    private static String calculateColorName(double[] color) {
        if (color == null || color.length < 3) {
            return "Unknown";
        }

        double y = color[0]; // Y component
        double b = color[1]; // B component
        double r = color[2]; // R component

        // Logic to determine color name (tune thresholds as necessary)
        if (r > b && r > y) {
            return "Red";
        } else if (b > r && b > y) {
            return "Blue";
        } else if (y > r && y > b) {
            return "Yellow";
        }

        return "Unknown"; // Default if no dominant color is found
    }
}