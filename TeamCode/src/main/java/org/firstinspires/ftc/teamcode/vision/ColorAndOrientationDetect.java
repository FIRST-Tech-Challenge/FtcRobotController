package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorAndOrientationDetect implements VisionProcessor {

    // Color ranges for blue
    private final Scalar lowerBlue = new Scalar(55.3, 90.7, 141.7);  // Blue lower bound
    private final Scalar upperBlue = new Scalar(181.3, 182.8, 255.0); // Blue upper bound

    // Color ranges for red
    private final Scalar lowerRed = new Scalar(19.8, 164.0, 68.0);   // Red lower bound
    private final Scalar upperRed = new Scalar(171.4, 200.0, 120.0); // Red upper bound

    // Color ranges for yellow
    private final Scalar lowerYellow = new Scalar(79.3, 140.0, 20.0);  // Yellow lower bound
    private final Scalar upperYellow = new Scalar(204.0, 225.0, 75.0); // Yellow upper bound

    // Minimum bounding box area (filter out small objects)
    private double MIN_BOUNDING_BOX_AREA = 0.05 * 640 * 480;  // 5% of frame size (640x480)

    // List to store detected colors with angles
    private final List<DetectedAngle> detectedColors = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize variables or settings if needed
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        detectedColors.clear(); // Clear previous detections

        try {
            // Convert the input frame to the YCrCb and HSV color spaces
            Mat ycrcb = new Mat();
            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Process each mask (blue, red, yellow)
            processMask(ycrcb, hsvMat, input, lowerBlue, upperBlue, "Blue", new Scalar(255, 0, 0)); // Blue
            processMask(ycrcb, hsvMat, input, lowerRed, upperRed, "Red", new Scalar(0, 0, 255));   // Red
            processMask(ycrcb, hsvMat, input, lowerYellow, upperYellow, "Yellow", new Scalar(0, 255, 255)); // Yellow

            // Release resources
            ycrcb.release();
            hsvMat.release();
        } catch (Exception e) {
            e.printStackTrace(); // Log unexpected errors
        }

        // Return the modified frame with annotations
        return input;
    }

    private void processMask(Mat ycrcb, Mat hsvMat, Mat frame, Scalar lower, Scalar upper, String colorName, Scalar boxColor) {
        Mat mask = new Mat();
        Core.inRange(ycrcb, lower, upper, mask);

        try {
            processOrientation(mask, hsvMat, frame, colorName, boxColor);
        } finally {
            mask.release(); // Release the mask to free memory
        }
    }

    private void processOrientation(Mat mask, Mat hsvMat, Mat frame, String colorName, Scalar boxColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        try {
            Imgproc.findContours(mask.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double boundingBoxArea = rect.width * rect.height;

                // Filter out small contours
                if (boundingBoxArea > MIN_BOUNDING_BOX_AREA) {
                    double avgSaturation = getAvgSaturation(hsvMat, rect);
                    double angle = getAngle(contour);
                    Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0); // Center of bounding box

                    // Draw the bounding box
                    Imgproc.rectangle(frame, rect, boxColor, 2);

                    // Draw the color name above the bounding box
                    Imgproc.putText(frame, colorName,
                            new Point(rect.x, rect.y - 10), // Position above the bounding box
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5, // Font size
                            new Scalar(0, 255, 0), // White color for text
                            1); // Thickness

                    // Draw the angle below the bounding box
                    Imgproc.putText(frame, "Angle: " + String.format("%.1f", angle),
                            new Point(rect.x, rect.y + rect.height + 20), // Position below the bounding box
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            new Scalar(0, 255, 0),
                            1);

                    // Draw the center coordinates below the angle
                    Imgproc.putText(frame, "Center: (" + String.format("%.1f", center.x) + ", " + String.format("%.1f", center.y) + ")",
                            new Point(rect.x, rect.y + rect.height + 40), // Position below the angle
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            new Scalar(0, 255, 0),
                            1);

                    // Add the detected object to the list
                    double[] detectedColor = {avgSaturation, 0, 0}; // Replace with actual logic for [Y, B, R]
                    DetectedAngle detectedAngle = new DetectedAngle(rect, boundingBoxArea, angle, detectedColor);
                    detectedColors.add(detectedAngle);
                }
            }
        } finally {
            // Release resources
            hierarchy.release();
            for (MatOfPoint contour : contours) {
                contour.release();
            }
        }
    }

    protected double getAvgSaturation(Mat hsvInput, Rect rect) {
        Mat submat = hsvInput.submat(rect);
        try {
            Scalar meanVal = Core.mean(submat);
            return meanVal.val[1]; // Return the saturation channel value
        } finally {
            submat.release();
        }
    }

    public void setMinBoundingBoxArea(double ratio) {
        if (ratio <= 0 || ratio > 1) {
            throw new IllegalArgumentException("Ratio must be between 0 and 1.");
        }
        this.MIN_BOUNDING_BOX_AREA = ratio * 640 * 480; // Adjust minimum bounding box area
    }

    private double getAngle(MatOfPoint contour) {
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
        try {
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
            double angle = rotatedRect.angle;
            if (rotatedRect.size.width < rotatedRect.size.height) {
                angle += 90;
            }
            return angle;
        } finally {
            contour2f.release();
        }
    }

    // Public Method: Get angle for the specified color
    public Double calAngle(String colorName) {
        if (detectedColors.isEmpty()) {
            return null; // Return null if no objects are detected
        }

        for (DetectedAngle detected : detectedColors) {
            if (detected.getColorName().equalsIgnoreCase(colorName)) {
                return detected.getAngle(); // Return the angle for the first detected object of the specified color
            }
        }

        return null; // Return null if no object of the specified color is found
    }

    // Public Method: Get center coordinates of the specified color
    public double[] calCenter(String colorName) {
        if (detectedColors.isEmpty()) {
            return null; // Return null if nothing is detected
        }

        for (DetectedAngle detected : detectedColors) {
            if (detected.getColorName().equalsIgnoreCase(colorName)) {
                Point center = detected.getCenter();
                return new double[]{center.x, center.y};
            }
        }

        return null; // Return null if the color is not found
    }

    // Public Method: Check if a specific color is detected
    public boolean isColorExist(String colorName) {
        if (detectedColors.isEmpty() || colorName == null || colorName.isEmpty()) {
            return false; // No objects detected or invalid input
        }

        for (DetectedAngle detected : detectedColors) {
            if (detected.getColorName().equalsIgnoreCase(colorName)) {
                return true;
            }
        }

        return false;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Optional: Implement if drawing on a Canvas is needed
    }
}
