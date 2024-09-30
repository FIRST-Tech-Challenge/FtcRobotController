package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class FastDetectSamples extends OpenCvPipeline {
    public /*final*/ OpenCvCamera webcam;
    boolean viewportPaused;

    public double fov = 78;

    private final Telemetry telemetry;

    public FastDetectSamples(/*OpenCvCamera webcam*/ Telemetry telemetry){
        //this.webcam = webcam;
        this.telemetry = telemetry;
    }
    public Mat processFrame(Mat input) {
        Mat yellowMask = preprocessFrame(input);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Mat yosi = Mat.zeros(input.size(), input.type());
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Calculate the epsilon (accuracy parameter)
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);

            MatOfPoint2f contour_approx = new MatOfPoint2f(contour.toArray());
            Imgproc.approxPolyDP(contour2f, contour_approx, epsilon, true);
            MatOfPoint points = new MatOfPoint(contour_approx.toArray());

            Point[] vertices = points.toArray();

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
    @Override
    public void onViewportTapped()
    {
        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
         */

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
