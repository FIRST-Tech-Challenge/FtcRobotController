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
            MatOfInt4 lines = new MatOfInt4();
            Mat black = Mat.zeros(input.size(), input.type());

            MatOfPoint2f contour_approx = new MatOfPoint2f(contour.toArray());

            Imgproc.approxPolyDP(contour_approx, contour_approx, 0.4, true);
            MatOfPoint approx = new MatOfPoint(contour_approx.toArray());
            List<MatOfPoint> approxContours = new ArrayList<>();
            approxContours.add(approx);
            Imgproc.cvtColor(black, black, Imgproc.COLOR_RGB2GRAY);
            Imgproc.drawContours(black, approxContours, -1, new Scalar(255, 255, 255), 1);
            Imgproc.HoughLinesP(black, lines, 2, Math.PI / 180, 15, 10, 25);
            try {
                int[] linesArray = lines.toArray();
                telemetry.addData("Number of lines", linesArray.length / 4);
                linesArray = lines.toArray();
                for (int i = 0; i < linesArray.length; i += 4) {
                    int x1 = linesArray[i], y1 = linesArray[i + 1], x2 = linesArray[i + 2], y2 = linesArray[i + 3];
                    if (Math.abs(x1 - x2) < 7) { // Adjust threshold for verticality
                        double length = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
                        Imgproc.line(input, new Point(x1, y1), new Point(x2, y2), new Scalar(0, 0, 0), 1);
                        Imgproc.putText(input, String.valueOf(Math.round(length)), new Point((x1 + x2) / 2, (y1 + y2) / 2), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 0), 1);
                        telemetry.addData("Vertical Line Length", length);
                        }
                }
            }
            catch (Exception e) {
                telemetry.addData("No lines found", "");
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
