package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class EdgeDetectionPipeline extends OpenCvPipeline {
    // Mat objects for processing
    private Mat grayMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat contoursMat = new Mat();

    // Angle of the detected sample
    private double detectedAngle = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to grayscale
        Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_RGB2GRAY);

        // Apply binary thresholding to isolate the sample
        Imgproc.threshold(grayMat, thresholdMat, 128, 255, Imgproc.THRESH_BINARY);

        // Find contours in the thresholded image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours and calculate angle
        contoursMat = input.clone();
        for (MatOfPoint contour : contours) {
            // Approximate the contour to a polygon
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            // Draw the rectangle around the sample
            Point[] rectPoints = new Point[4];
            rotatedRect.points(rectPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(contoursMat, rectPoints[i], rectPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Get the angle of the sample
            detectedAngle = rotatedRect.angle;
            if (rotatedRect.size.width < rotatedRect.size.height) {
                detectedAngle += 90; // Adjust for vertical orientation
            }

            // Display the angle on the image
            Imgproc.putText(
                    contoursMat,
                    String.format("Angle: %.2f", detectedAngle),
                    rotatedRect.center,
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    new Scalar(255, 0, 0),
                    2
            );
        }

        // Return the image with contours and angle annotations
        return contoursMat;
    }

    /**
     * Get the detected angle of the sample.
     *
     * @return The angle in degrees.
     */
    public double getDetectedAngle() {
        return detectedAngle;
    }
}
