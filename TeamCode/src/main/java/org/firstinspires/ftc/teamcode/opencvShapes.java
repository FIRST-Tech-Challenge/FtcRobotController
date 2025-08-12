package org.firstinspires.ftc.teamcode.vision;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class opencvShapes extends OpenCvPipeline {
    private Mat gray = new Mat();
    private Mat blurred = new Mat();
    private Mat edges = new Mat();
    private Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Convert to grayscale
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        // Blur to reduce noise
        Imgproc.GaussianBlur(gray, blurred, new Size(5, 5), 0);

        // Edge detection
        Imgproc.Canny(blurred, edges, 75, 200);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea < 500) continue; // Ignore small noise

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double perimeter = Imgproc.arcLength(contour2f, true);

            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, 0.04 * perimeter, true);

            int vertices = (int) approx.total();

            Rect boundingRect = Imgproc.boundingRect(contour);

            // Circle detection (approximation: many vertices & aspect ratio near 1)
            if (vertices > 8) {
                Imgproc.circle(input, new Point(boundingRect.x + boundingRect.width / 2.0, boundingRect.y + boundingRect.height / 2.0),
                        (int) (boundingRect.width / 2.0), new Scalar(255, 0, 0), 3);
                Imgproc.putText(input, "Circle", boundingRect.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 0, 0), 2);
            }
            // Triangle
            else if (vertices == 3) {
                Imgproc.drawContours(input, List.of(new MatOfPoint(approx.toArray())), -1, new Scalar(0, 255, 0), 3);
                Imgproc.putText(input, "Triangle", boundingRect.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0, 255, 0), 2);
            }
            // Rectangle
            else if (vertices == 4) {
                Imgproc.drawContours(input, List.of(new MatOfPoint(approx.toArray())), -1, new Scalar(0, 0, 255), 3);
                Imgproc.putText(input, "Rectangle", boundingRect.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0, 0, 255), 2);
            }
            // Hexagon
            else if (vertices == 6) {
                Imgproc.drawContours(input, List.of(new MatOfPoint(approx.toArray())), -1, new Scalar(255, 255, 0), 3);
                Imgproc.putText(input, "Hexagon", boundingRect.tl(), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 0), 2);
            }
        }

        return input;
    }
}
