package org.firstinspires.ftc.teamcode2024;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RedZoneDetector extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define lower and upper bounds for red color in HSV
        Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(10, 255, 255);

        // Create a binary mask for red regions
        Mat mask = new Mat();
        Core.inRange(hsvImage, lowerRed, upperRed, mask);

        // Find contours in the binary mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the contour with the largest area (zone with the most red)
        double maxArea = 0;
        MatOfPoint maxContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxContour = contour;
            }
        }

        // Draw a circle around the identified zone
        if (maxContour != null) {
            Moments moments = Imgproc.moments(maxContour);
            int cx = (int) (moments.get_m10() / moments.get_m00());
            int cy = (int) (moments.get_m01() / moments.get_m00());

            int radius = 20; // You can adjust the radius as needed
            Imgproc.circle(input, new Point(cx, cy), radius, new Scalar(0, 255, 0), 2);
        }

        // Return the processed frame
        return input;
    }
}
