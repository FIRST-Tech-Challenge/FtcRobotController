package org.firstinspires.ftc.teamcode.opencv;
import org.opencv.core.Core;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class DetectSamples extends OpenCvPipeline {
    public /*final*/ OpenCvCamera webcam;
    boolean viewportPaused;

    private final Telemetry telemetry;

    public DetectSamples(/*OpenCvCamera webcam*/ Telemetry telemetry){
        //this.webcam = webcam;
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat yellowMask = preprocessFrame(input);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        List<Double> verticalLineLenghs = new ArrayList<Double>();
        for (MatOfPoint contour : contours) {
            Mat canvas = Mat.zeros(input.size(), input.type());
            Imgproc.drawContours(canvas, contours, -1, new Scalar(255, 255, 255), -1);
            Core.bitwise_and(input, canvas, canvas);
            Mat edges = new Mat();
            Imgproc.Canny(canvas, edges, 100, 190, 3, true);
            MatOfInt4 lines = new MatOfInt4();
            Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 0, 0, 0);
            int[] linesToArray = lines.toArray();
        }
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 255, 255), -1);
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