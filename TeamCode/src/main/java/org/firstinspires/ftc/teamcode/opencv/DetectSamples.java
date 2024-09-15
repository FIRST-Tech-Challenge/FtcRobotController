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
        List<Double> verticalLineLengths = new ArrayList<Double>();
        Mat black = Mat.zeros(input.size(), input.type());
        Imgproc.drawContours(black, contours, -1, new Scalar(255, 255, 255), -1);
        Mat yosi = new Mat();
        Core.bitwise_and(input, black, yosi);

        Mat edges = new Mat();
        Imgproc.Canny(yosi, edges, 100, 190, 3, true);
        Mat show_lines = Mat.zeros(input.size(), input.type());
        for (MatOfPoint contour : contours) {
            MatOfInt4 lines = new MatOfInt4();
            Rect boundingRect = Imgproc.boundingRect(contour);
            int padding = 5; // Add padding of 5 pixels to capture edges near the boundary
            Rect expandedRect = new Rect(
                    Math.max(boundingRect.x - padding, 0),
                    Math.max(boundingRect.y - padding, 0),
                    Math.min(boundingRect.width + 2 * padding, input.cols() - boundingRect.x),
                    Math.min(boundingRect.height + 2 * padding, input.rows() - boundingRect.y)
            );
            Mat roiEdges = new Mat(edges, expandedRect);
            Imgproc.dilate(edges, edges, new Mat(), new Point(0, -1), 1);
            Imgproc.erode(edges, edges, new Mat(), new Point(-1, -1), 1);
            Imgproc.HoughLinesP(roiEdges, lines, 1, Math.PI / 180, 12, 10, 10);
            telemetry.addData("Number of lines", lines.rows());

            int[] linesArray;
            try {
                linesArray = lines.toArray();
                for (int i = 0; i < linesArray.length; i += 4) {
                    int x1 = linesArray[i], y1 = linesArray[i + 1], x2 = linesArray[i + 2], y2 = linesArray[i + 3];
                    // Check if the line is vertical (x1 ~ x2)
                    if (Math.abs(x1 - x2) < 7) { // Adjust threshold for verticality
                        double length = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
                        Imgproc.line(show_lines, new Point(x1 + expandedRect.x, y1 + expandedRect.y),new Point(x2 + expandedRect.x, y2 + expandedRect.y), new Scalar(255, 255, 255), 1);
                        verticalLineLengths.add(length);

                        telemetry.addData("Vertical Line Length", length);
                    }
                }
            }

            catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }

            telemetry.update();

        }
        /*

        */
        return show_lines;
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