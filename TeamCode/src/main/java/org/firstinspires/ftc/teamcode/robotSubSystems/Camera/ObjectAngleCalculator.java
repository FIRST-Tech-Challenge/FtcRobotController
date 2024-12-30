package org.firstinspires.ftc.teamcode.robotSubSystems.Camera;
import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;

public class ObjectAngleCalculator implements VisionProcessor {
   public static double angle = 0;
    public static double getAngle(Mat frame) {
        // Load image


        // Detect edges using Canny edge detection
        Mat edges = new Mat();
        Imgproc.Canny(frame, edges, 100, 200);

        // Find contours in the edge image
        Mat hierarchy = new Mat();
        java.util.List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour (assuming it's our object)
        if (!contours.isEmpty()) {
            MatOfPoint largestContour = contours.get(0);
            double maxArea = Imgproc.contourArea(largestContour);

            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > maxArea) {
                    largestContour = contour;
                    maxArea = Imgproc.contourArea(contour);
                }
            }

            // Calculate moments
            MatOfDouble mu = new MatOfDouble();

            Imgproc.moments(largestContour, false);
            Imgproc.moments(largestContour, false);

            double m10 = mu.get(1, 0)[0];
            double m01 = mu.get(0, 1)[0];

            // Calculate centroid
            Point centroid = new Point(m10 /  maxArea, m01 /  maxArea);

            // Calculate angle
            int centerX = frame.cols() / 2;
            int centerY = frame.rows() / 2;
            angle = Math.atan2(centroid.y - centerY, centroid.x - centerX);



            // Convert to degrees
            angle = Math.toDegrees(angle);


        }
        return angle;
    }

    @Override
    public void init(int i, int i1, CameraCalibration cameraCalibration) {

    }

    @Override
    public Object processFrame(Mat mat, long l) {
        getAngle(mat);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

    }
}
