package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectionPipeline extends OpenCvPipeline {

    private String detectedColor = "None";
    private Point detectedPosition = new Point(-1, -1);

    // HSV thresholds for colors (H, S, V)
    // NOTE: FTC OpenCV uses HSV ranges: H: 0–179, S: 0–255, V: 0–255
    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    private final Scalar lowerRed1 = new Scalar(0, 100, 100);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);
    private final Scalar lowerRed2 = new Scalar(160, 100, 100);
    private final Scalar upperRed2 = new Scalar(179, 255, 255);

    private final Scalar lowerBlue = new Scalar(100, 100, 100);
    private final Scalar upperBlue = new Scalar(140, 255, 255);

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat maskYellow = new Mat();
        Mat maskRed1 = new Mat();
        Mat maskRed2 = new Mat();
        Mat maskRed = new Mat();
        Mat maskBlue = new Mat();

        // Yellow mask
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

        // Red mask (two ranges combined)
        Core.inRange(hsv, lowerRed1, upperRed1, maskRed1);
        Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
        Core.addWeighted(maskRed1, 1.0, maskRed2, 1.0, 0.0, maskRed);

        // Blue mask
        Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

        // Find largest contour for each color
        double yellowArea = Core.countNonZero(maskYellow);
        double redArea = Core.countNonZero(maskRed);
        double blueArea = Core.countNonZero(maskBlue);

        // Pick the largest color area
        double maxArea = Math.max(yellowArea, Math.max(redArea, blueArea));

        if (maxArea == 0) {
            detectedColor = "None";
            detectedPosition = new Point(-1, -1);
        } else if (maxArea == yellowArea) {
            detectedColor = "Yellow";
            detectedPosition = findCenter(maskYellow);
            Imgproc.circle(input, detectedPosition, 10, new Scalar(255, 255, 0), 2);
        } else if (maxArea == redArea) {
            detectedColor = "Red";
            detectedPosition = findCenter(maskRed);
            Imgproc.circle(input, detectedPosition, 10, new Scalar(255, 0, 0), 2);
        } else if (maxArea == blueArea) {
            detectedColor = "Blue";
            detectedPosition = findCenter(maskBlue);
            Imgproc.circle(input, detectedPosition, 10, new Scalar(0, 0, 255), 2);
        }

        // Cleanup
        hsv.release();
        maskYellow.release();
        maskRed1.release();
        maskRed2.release();
        maskRed.release();
        maskBlue.release();

        return input;
    }

    private Point findCenter(Mat mask) {
        // Find contours from the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            // Get the largest contour
            MatOfPoint largestContour = contours.get(0);
            double maxArea = Imgproc.contourArea(largestContour);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            // Get bounding rectangle of largest contour
            Rect rect = Imgproc.boundingRect(largestContour);
            return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
        }

        // No contours found
        return new Point(-1, -1);
    }


    // Methods to get telemetry info
    public String getDetectedColor() {
        return detectedColor;
    }

    public Point getDetectedPosition() {
        return detectedPosition;
    }
}
