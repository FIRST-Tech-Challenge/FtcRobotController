package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import java.util.Collections;
import java.util.concurrent.atomic.AtomicLong;

public class ColorAndOrientationDetect implements VisionProcessor {

    // Color ranges
    private final Scalar lowerBlue = new Scalar(55.3, 90.7, 141.7);  // Blue lower bound
    private final Scalar upperBlue = new Scalar(181.3, 182.8, 255.0); // Blue upper bound

    private final Scalar lowerRed = new Scalar(19.8, 164.0, 68.0);   // Red lower bound
    private final Scalar upperRed = new Scalar(171.4, 200.0, 120.0); // Red upper bound

    private final Scalar lowerYellow = new Scalar(79.3, 140.0, 20.0);  // Yellow lower bound
    private final Scalar upperYellow = new Scalar(204.0, 225.0, 75.0); // Yellow upper bound

    // Minimum bounding box area (filter out small objects)
    private double MIN_BOUNDING_BOX_AREA = 0.05 * 640 * 480; // 5% of frame size (640x480)

    // Shared list for detected results
    private final List<DetectedAngle> detections = new ArrayList<>();
    private final Object lock = new Object(); // Synchronization lock for thread-safety
    private final AtomicLong lastUpdatedTime = new AtomicLong(0); // Thread-safe timestamp

    // Telemetry instance
    private Telemetry telemetry;

    // Constructor to pass Telemetry
//    public ColorAndOrientationDetect(Telemetry telemetry) {
//        this.telemetry = telemetry;
//    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize variables or settings if needed
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        // Clear the detections list for the current frame
        synchronized (lock) {
            detections.clear();
            lastUpdatedTime.set(System.currentTimeMillis()); // Update timestamp
        }

        try {
            // Convert the input frame to the YCrCb color space
            Mat ycrcb = new Mat();
            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

            // Process masks for blue, red, and yellow
            processMask(ycrcb, input, lowerBlue, upperBlue, "Blue", new Scalar(255, 0, 0));
            processMask(ycrcb, input, lowerRed, upperRed, "Red", new Scalar(0, 0, 255));
            processMask(ycrcb, input, lowerYellow, upperYellow, "Yellow", new Scalar(0, 255, 255));

            // Update telemetry with detection data
//            updateTelemetry();

            // Release resources
            ycrcb.release();
        } catch (Exception e) {
            e.printStackTrace();
        }

        return input; // Return the frame with annotations
    }

    public long getLastUpdatedTime() {
        return lastUpdatedTime.get();
    }

    private void processMask(Mat ycrcb, Mat frame, Scalar lower, Scalar upper, String colorName, Scalar boxColor) {
        Mat mask = new Mat();
        Core.inRange(ycrcb, lower, upper, mask);

        try {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double area = rect.width * rect.height;

                // Skip small contours
                if (area > MIN_BOUNDING_BOX_AREA) {
                    double angle = calculateAngle(contour);
                    DetectedAngle detected = new DetectedAngle(rect, area, angle, colorName);

                    // Add the detected result to the shared list
                    synchronized (lock) {
                        detections.add(detected);
                    }

                    // Draw bounding box and annotations on the frame
                    Imgproc.rectangle(frame, rect, boxColor, 2);
                    Imgproc.putText(frame, colorName, new Point(rect.x, rect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
                    Imgproc.putText(frame, "Angle: " + String.format("%.1f", angle),
                            new Point(rect.x, rect.y + rect.height + 20),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
                    // Draw the center coordinates below the angle
                    Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
                    Imgproc.putText(frame, "Center: (" + String.format("%.1f", center.x) + ", " + String.format("%.1f", center.y) + ")",
                            new Point(rect.x, rect.y + rect.height + 40), // Position below the angle
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            new Scalar(0, 255, 0),
                            1);
                }
            }

            mask.release();
            hierarchy.release();
        } finally {
            mask.release();
        }
    }

    private double calculateAngle(MatOfPoint contour) {
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

    public void setMinBoundingBoxArea(double ratio) {
        if (ratio <= 0 || ratio > 1) {
            throw new IllegalArgumentException("Ratio must be between 0 and 1.");
        }
        this.MIN_BOUNDING_BOX_AREA = ratio * 640 * 480; // Adjust minimum bounding box area
    }

    // Thread-safe public method to get angle for a specific color
    public double calAngle(String colorName) {
        synchronized (lock) {
            for (DetectedAngle detected : detections) {
                if (detected.getColorName().equalsIgnoreCase(colorName)) {
                    return detected.getAngle();
                }
            }
            return -1.0; // Not found
        }
    }

    // Thread-safe public method to get center for a specific color
    public double[] calCenter(String colorName) {
        synchronized (lock) {
            for (DetectedAngle detected : detections) {
                if (detected.getColorName().equalsIgnoreCase(colorName)) {
                    Point center = detected.getCenter();
                    return new double[]{center.x, center.y};
                }
            }
            return new double[0]; // Not found
        }
    }

    // Thread-safe public method to check if a specific color exists
    public boolean isColorExist(String colorName) {
        synchronized (lock) {
            for (DetectedAngle detected : detections) {
                if (detected.getColorName().equalsIgnoreCase(colorName)) {
                    return true;
                }
            }
            return false;
        }
    }

    // Update telemetry with current detections (refactored to use getDetectedColorAndAng)
//    private void updateTelemetry() {
//        // Fetch detected objects using getDetectedColorAndAng (thread-safe)
//        List<DetectedAngle> detectedObjects = getDetectedColorAndAng();
//
//        telemetry.clear(); // Clear old telemetry data
//        telemetry.addLine("Detected Objects:");
//
//        // Iterate over the detected objects and add them to telemetry
//        for (DetectedAngle detected : detectedObjects) {
//            telemetry.addData(
//                    detected.getColorName(),
//                    "Angle: %.1f, Center: (%.1f, %.1f)",
//                    detected.getAngle(),
//                    detected.getCenter().x,
//                    detected.getCenter().y
//            );
//        }
//
//        telemetry.update(); // Push telemetry to the driver station
//    }
//

    // Thread-safe public method to get all detected objects (colors, angles, and centers)
    public List<DetectedAngle> getDetectedColorAndAng() {
        synchronized (lock) {
            return Collections.unmodifiableList(new ArrayList<>(detections)); // Return an immutable copy
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Optional: Implement if drawing on a Canvas is needed
    }
}
