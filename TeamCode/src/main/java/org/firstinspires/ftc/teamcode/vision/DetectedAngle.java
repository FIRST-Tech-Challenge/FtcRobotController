package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;
import org.opencv.core.Rect;

public class DetectedAngle {
    private final String colorName; // Name of the detected color
    private final Rect boundingBox; // Bounding box of the detected object
    private final double area; // Area of the bounding box
    private final double angle; // Orientation angle of the detected object
    private final Point center; // Center of the bounding box

    public DetectedAngle(Rect boundingBox, double area, double angle, String colorName) {
        this.boundingBox = boundingBox;
        this.area = area;
        this.angle = angle;
        this.colorName = colorName;
        this.center = calculateCenter(boundingBox);
    }

    private Point calculateCenter(Rect boundingBox) {
        double centerX = boundingBox.x + boundingBox.width / 2.0;
        double centerY = boundingBox.y + boundingBox.height / 2.0;
        return new Point(centerX, centerY);
    }

    public String getColorName() {
        return colorName;
    }

    public Rect getBoundingBox() {
        return boundingBox;
    }

    public double getArea() {
        return area;
    }

    public double getAngle() {
        return angle;
    }

    public Point getCenter() {
        return center;
    }
}
