package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectionPipeline extends OpenCvPipeline {

    public enum DetectedColor {
        RED, BLUE, YELLOW, NONE
    }

    private DetectedColor detectedColor = DetectedColor.NONE;
    private Point objectCenter = new Point(-1, -1);

    // HSV ranges for colors (tune these for your lighting!)
    private Scalar lowerRed1 = new Scalar(0, 100, 100);
    private Scalar upperRed1 = new Scalar(10, 255, 255);
    private Scalar lowerRed2 = new Scalar(160, 100, 100);
    private Scalar upperRed2 = new Scalar(179, 255, 255);

    private Scalar lowerBlue = new Scalar(100, 150, 0);
    private Scalar upperBlue = new Scalar(140, 255, 255);

    private Scalar lowerYellow = new Scalar(20, 100, 100);
    private Scalar upperYellow = new Scalar(30, 255, 255);

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Create masks
        Mat redMask1 = new Mat();
        Mat redMask2 = new Mat();
        Mat redMask = new Mat();
        Mat blueMask = new Mat();
        Mat yellowMask = new Mat();

        Core.inRange(hsv, lowerRed1, upperRed1, redMask1);
        Core.inRange(hsv, lowerRed2, upperRed2, redMask2);
        Core.bitwise_or(redMask1, redMask2, redMask);

        Core.inRange(hsv, lowerBlue, upperBlue, blueMask);
        Core.inRange(hsv, lowerYellow, upperYellow, yellowMask);

        // Find the largest contour for each color
        double redArea = getLargestContourArea(redMask);
        double blueArea = getLargestContourArea(blueMask);
        double yellowArea = getLargestContourArea(yellowMask);

        // Choose the largest detected color
        double maxArea = Math.max(redArea, Math.max(blueArea, yellowArea));
        if (maxArea < 500) { // Ignore tiny noise
            detectedColor = DetectedColor.NONE;
            objectCenter = new Point(-1, -1);
        } else if (maxArea == redArea) {
            detectedColor = DetectedColor.RED;
            objectCenter = findCenter(redMask);
        } else if (maxArea == blueArea) {
            detectedColor = DetectedColor.BLUE;
            objectCenter = findCenter(blueMask);
        } else if (maxArea == yellowArea) {
            detectedColor = DetectedColor.YELLOW;
            objectCenter = findCenter(yellowMask);
        }

        // Draw center point on screen
        if (detectedColor != DetectedColor.NONE) {
            Imgproc.circle(input, objectCenter, 5, new Scalar(0, 255, 0), -1);
        }

        hsv.release();
        redMask1.release();
        redMask2.release();
        redMask.release();
        blueMask.release();
        yellowMask.release();

        return input;
    }

    private double getLargestContourArea(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
            }
        }
        return maxArea;
    }

    private Point findCenter(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            // Get largest contour
            MatOfPoint largestContour = contours.get(0);
            double maxArea = Imgproc.contourArea(largestContour);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            Rect rect = Imgproc.boundingRect(largestContour);
            return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
        }
        return new Point(-1, -1);
    }

    public DetectedColor getDetectedColor() {
        return detectedColor;
    }

    public Point getObjectCenter() {
        return objectCenter;
    }
}
