package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Moments;
import org.opencv.core.Core;

import java.util.List;
import java.util.ArrayList;

public class SampleDetectionPipeline extends OpenCvPipeline {

    // Public fields for telemetry
    public double yellowX = -1, yellowY = -1, yellowDist = -1;
    public double redX = -1, redY = -1, redDist = -1;
    public double blueX = -1, blueY = -1, blueDist = -1;

    private static final double KNOWN_WIDTH_INCHES = 2.0;
    private static final double FOCAL_LENGTH_PIXELS = 800.0;

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // Masks
        Mat yellowMask = getYellowMask(hsv);
        Mat redMask = getRedMask(hsv);
        Mat blueMask = getBlueMask(hsv);

        // Detect & draw each color
        yellowDist = detectAndDraw(input, yellowMask, new Scalar(0, 255, 255), "YELLOW", 0, v -> yellowX = v[0], v -> yellowY = v[0]);
        redDist    = detectAndDraw(input, redMask,    new Scalar(0, 0, 255),   "RED",    20, v -> redX = v[0],    v -> redY = v[0]);
        blueDist   = detectAndDraw(input, blueMask,   new Scalar(255, 0, 0),   "BLUE",   40, v -> blueX = v[0],   v -> blueY = v[0]);

        return input;
    }

    private double detectAndDraw(Mat frame, Mat mask, Scalar color, String label, int yOffset,
                                 java.util.function.Consumer<double[]> setX,
                                 java.util.function.Consumer<double[]> setY) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double distance = -1;
        if (!contours.isEmpty()) {
            MatOfPoint largestContour = contours.get(0);
            double maxArea = Imgproc.contourArea(largestContour);
            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = c;
                }
            }

            Rect rect = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(frame, rect, color, 2);

            Moments m = Imgproc.moments(largestContour);
            double cX = -1, cY = -1;
            if (m.get_m00() != 0) {
                cX = m.get_m10() / m.get_m00();
                cY = m.get_m01() / m.get_m00();
                Imgproc.circle(frame, new Point(cX, cY), 5, color, -1);
            }

            // Update telemetry fields
            setX.accept(new double[]{cX});
            setY.accept(new double[]{cY});

            // Distance calculation
            distance = getDistance(rect.width);
            distance = Math.round(distance * 100.0) / 100.0;

            Imgproc.putText(frame, label, new Point(rect.x, rect.y - 10 - yOffset), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            Imgproc.putText(frame, "Dist: " + distance + " in", new Point(rect.x, rect.y + rect.height + 15 + yOffset), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
        return distance;
    }

    private double getDistance(double pixelWidth) {
        if (pixelWidth > 0) {
            return (KNOWN_WIDTH_INCHES * FOCAL_LENGTH_PIXELS) / pixelWidth;
        }
        return -1;
    }

    private Mat getYellowMask(Mat hsv) {
        Scalar lower = new Scalar(20, 100, 100);
        Scalar upper = new Scalar(30, 255, 255);
        return getCleanMask(hsv, lower, upper);
    }

    private Mat getBlueMask(Mat hsv) {
        Scalar lower = new Scalar(100, 150, 0);
        Scalar upper = new Scalar(140, 255, 255);
        return getCleanMask(hsv, lower, upper);
    }

    private Mat getRedMask(Mat hsv) {
        // Lower red range
        Scalar lower1 = new Scalar(0, 120, 70);
        Scalar upper1 = new Scalar(10, 255, 255);
        Mat mask1 = new Mat();
        Core.inRange(hsv, lower1, upper1, mask1);

        // Upper red range
        Scalar lower2 = new Scalar(170, 120, 70);
        Scalar upper2 = new Scalar(180, 255, 255);
        Mat mask2 = new Mat();
        Core.inRange(hsv, lower2, upper2, mask2);

        Mat redMask = new Mat();
        Core.add(mask1, mask2, redMask);
        return cleanMask(redMask);
    }

    private Mat getCleanMask(Mat hsv, Scalar lower, Scalar upper) {
        Mat mask = new Mat();
        Core.inRange(hsv, lower, upper, mask);
        return cleanMask(mask);
    }

    private Mat cleanMask(Mat mask) {
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        return mask;
    }
}
