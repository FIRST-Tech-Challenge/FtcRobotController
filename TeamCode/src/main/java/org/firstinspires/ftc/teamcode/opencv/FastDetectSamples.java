package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class FastDetectSamples extends OpenCvPipeline {
    public /*final*/ OpenCvCamera webcam;
    boolean viewportPaused;

    public double h_fov = 78;
    public double v_hov = 41.4;
    public double inchLength = 1.5;
    public double focalLength = -1;
    public double imageWidth = 1920;
    public double imageHeight = 1080;
    public double sampleZ = -1;
    private final Telemetry telemetry;

    public FastDetectSamples(/*OpenCvCamera webcam*/ Telemetry telemetry){
        //this.webcam = webcam;
        this.telemetry = telemetry;
    }
    public Mat processFrame(Mat input) {
        Mat yellowMask = preprocessFrame(input);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Calculate the epsilon (accuracy parameter)
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);

            MatOfPoint2f contour_approx = new MatOfPoint2f(contour.toArray());
            Imgproc.approxPolyDP(contour2f, contour_approx, epsilon, true);
            MatOfPoint points = new MatOfPoint(contour_approx.toArray());

            Point[] vertices = points.toArray();
            telemetry.addData("Vertices", vertices.length);
            for (int i = 0; i < vertices.length; i++) {
                //telemetry.addData(String.valueOf(i), "");
                Imgproc.circle(input, vertices[i], 2, new Scalar(0, 255, 0), 1);
                Point start = vertices[i];
                Point end = vertices[(i + 1) % vertices.length];
                double length = Math.sqrt(Math.pow(start.x - end.x, 2) + Math.pow(start.y - end.y, 2));
                if (Math.abs(start.x - end.x) < 0.2 * length) {
                    Imgproc.line(input, start, end, new Scalar(0, 255, 0), 1);
                    Imgproc.putText(input, Math.round(length) + "", new Point((start.x + end.x) / 2, (start.y + end.y) / 2), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 1);
                }
            }

        }
        telemetry.update();
        return input;
    }
    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowerYellow = new Scalar(0, 138, 0);
        Scalar upperYellow = new Scalar(255, 200, 100);


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }
    public double calculateX(double pixelLength) {
        return  inchLength * focalLength / pixelLength;
    }
    public double calculateY(double verticalAngle, double sampleX) {
        return  Math.tan(Math.toRadians(verticalAngle)) * sampleX;
    }
    public double calculateHorizontalAngle(double center_x) {
        return (center_x - imageWidth / 2) * h_fov / imageWidth;
    }
    public double calculateVerticalAngle(double center_y) {
        return (center_y - imageHeight / 2) * v_hov / imageHeight;
    }

    @Override
    public void onViewportTapped()
    {
        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }
}
