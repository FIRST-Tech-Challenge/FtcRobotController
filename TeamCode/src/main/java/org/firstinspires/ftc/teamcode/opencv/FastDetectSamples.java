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

public class FastDetectSamples extends OpenCvPipeline {
    public /*final*/ OpenCvCamera webcam;
    boolean viewportPaused;

    public double fov = 78;

    private final Telemetry telemetry;

    public FastDetectSamples(Telemetry telemetry){
        this.telemetry = telemetry;
    }


    public static int[] cleanLines(int[] lst, int noise){
        Point[] points = new Point[lst.length/2];
        int[] max_points = new int[lst.length];
        int[] indecies = new int[lst.length/4];
        int lines = 0;

        for (int i = 0; i < points.length; i++){
            points[i] = new Point(lst[2*i], lst[2*i+1]);
        }

        for (int i = 0; i < points.length; i += 2){
            if (indecies[i/2] != -1){
                Rect temp = new Rect(points[i], points[i+1]);
                temp.width += noise;
                temp.height += noise;
                for (int j = i + 2; j < points.length; j += 2){
                    if (temp.contains(points[j])){
                        indecies[j/2] = -1;
                        if (!temp.contains(points[j+1])){
                            temp.width -= noise;
                            temp.height -= noise;
                            Rect temp1 = new Rect(points[i], points[j+1]);
                            Rect temp2 = new Rect(points[j+1], points[i+1]);
                            if (temp1.area() > temp2.area()){
                                if (temp.area() < temp1.area()){
                                    temp = temp1;
                                }
                            }
                            else if (temp.area() < temp2.area()){
                                temp = temp2;
                            }
                            temp.width += noise;
                            temp.height += noise;
                        }
                    }
                    else if (temp.contains(points[j+1])){
                        indecies[j/2] = -1;
                        if (!temp.contains(points[j])){
                            temp.width -= noise;
                            temp.height -= noise;
                            Rect temp1 = new Rect(points[i], points[j]);
                            Rect temp2 = new Rect(points[j], points[i+1]);
                            if (temp1.area() > temp2.area()){
                                if (temp.area() < temp1.area()){
                                    temp = temp1;
                                }
                            }
                            else if (temp.area() < temp2.area()){
                                temp = temp2;
                            }
                            temp.width += noise;
                            temp.height += noise;
                        }
                    }
                }
                max_points[lines] = temp.x;
                max_points[lines+1] = temp.y;
                max_points[lines+2] = temp.x + temp.width;
                max_points[lines+3] = temp.y + temp.height;
                lines += 4;
            }
        }

        int[] result = new int[lines];
        for (int i = 0; i < result.length; i++){
            result[i] = max_points[i];
        }
        return result;
    }



    public Mat processFrame(Mat input) {
        Mat yellowMask = preprocessFrame(input);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(yellowMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            MatOfInt4 lines = new MatOfInt4();
            Mat black = Mat.zeros(input.size(), input.type());

            MatOfPoint2f contour_approx = new MatOfPoint2f(contour.toArray());

            Imgproc.approxPolyDP(contour_approx, contour_approx, 8, true);
            MatOfPoint approx = new MatOfPoint(contour_approx.toArray());
            List<MatOfPoint> approxContours = new ArrayList<>();
            approxContours.add(approx);
            Imgproc.cvtColor(black, black, Imgproc.COLOR_RGB2GRAY);
            Imgproc.drawContours(black, approxContours, -1, new Scalar(255, 255, 255), 1);
            Imgproc.HoughLinesP(black, lines, 10, Math.PI / 90, 12, 35, 40);
            boolean check = false;
            try {
                int[] linesArray = lines.toArray();
                check = true;
            }
            catch (Exception e) {
                telemetry.addData("No lines found", "");
            }
            if (check) {
                int[] linesArray = cleanLines(lines.toArray(), 5);

                telemetry.addData("Number of lines", linesArray.length / 4);

                for (int i = 0; i < linesArray.length; i += 4) {
                    int x1 = linesArray[i], y1 = linesArray[i + 1], x2 = linesArray[i + 2], y2 = linesArray[i + 3];
                    double length = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
                    if (Math.abs(x1 - x2) != length / 5) { // Adjust threshold for vertically
                        Imgproc.line(input, new Point(x1, y1), new Point(x2, y2), new Scalar(0, 0, 0), 1);
                        Imgproc.putText(input, String.valueOf(Math.round(length)), new Point((x1 + x2) / 2.0, (y1 + y2) / 2.0), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 0), 1);
                        telemetry.addData("Vertical Line Length", length);
                        }
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
