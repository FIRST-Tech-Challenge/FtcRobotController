package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;
import org.opencv.core.Rect;

public class DetectedAngle {
    private final String colorName; // Name of the detected color
    private final Rect boundingBox; // Bounding box around the detected color
    private final double area; // Area of the bounding box
    private final double angle; // Orientation angle of the detected color
    private final double[] color; // Detected color values [Y, B, R]
    private final Point center; // Center of the bounding box

    // Constructor
    public DetectedAngle(Rect boundingBox, double area, double angle, double[] color) {
        this.colorName = calculateColorName(color); // Infer color name from the color array
        this.boundingBox = boundingBox;
        this.area = area;
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

    // Getter for colorName
    public String getColorName() {
        return colorName;
    }

    // Getter for boundingBox
    public Rect getBoundingBox() {
        return boundingBox;
    }

    // Getter for angle
    public double getAngle() {
        return angle;
    }

    // Getter for center
    public Point getCenter() {
        return center;
    }

    // Method to calculate the color name based on the color array [Y, B, R]
    private static String calculateColorName(double[] color) {
        if (color == null || color.length < 3) {
            return "Unknown";
        }

        double y = color[0]; // Y component
        double b = color[1]; // B component
        double r = color[2]; // R component

        // Logic to determine the color name (adjust thresholds as needed)
        if (r > b && r > y) {
            return "Red";
        } else if (b > r && b > y) {
            return "Blue";
        } else if (y > r && y > b) {
            return "Yellow";
        }

        return "Unknown"; // Default if no dominant color is found
    }

    // Override toString for debugging purposes
    @Override
    public String toString() {
        return "DetectedAngle{" +
                "colorName='" + colorName + '\'' +
                ", boundingBox=" + boundingBox +
                ", area=" + area +
                ", angle=" + angle +
                ", color=[Y=" + color[0] + ", B=" + color[1] + ", R=" + color[2] + "]" +
                ", center=(" + center.x + ", " + center.y + ")" +
                '}';
    }
}
