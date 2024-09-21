package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FastDetectSamples extends OpenCvPipeline {

    public /*final*/ OpenCvCamera webcam;
    boolean viewportPaused;

    private final Telemetry telemetry;

    static int[] linesArray;

    //public final int fov = 78;

    public FastDetectSamples(Telemetry telemetry){
        this.telemetry = telemetry;
    }



    public static void cleanLines(int noise){
        int[] result = new int[linesArray.length];
        int[] indices = new int[linesArray.length/4];
        int values = 0;

        for (int i = 0; i < linesArray.length; i += 4){
            if (indices[i/2] != -1){
                Rectangle outer = new Rectangle(linesArray[i], linesArray[i+1], linesArray[i+2], linesArray[i+3]);
                outer.width += noise;
                outer.height += noise;
                for (int j = i + 2; j < linesArray.length; j += 4){
                    if (outer.contains(linesArray[j], linesArray[j+1])){
                        indices[j/2] = -1;
                        if (!outer.contains(linesArray[j+2], linesArray[j+3])){
                            outer = outer.fit(outer, i, j + 2, noise);
                        }
                    }
                    else if (outer.contains(linesArray[j+2], linesArray[j+3])){
                        indices[j/2] = -1;
                        if (!outer.contains(linesArray[j], linesArray[j+1])){
                            outer = outer.fit(outer, i, j, noise);
                        }
                    }
                }
                result[values] = outer.x;
                result[values+1] = outer.y;
                result[values+2] = outer.x + outer.width - noise;
                result[values+3] = outer.y + outer.height - noise;
                values += 4;
            }
        }

        linesArray =  Arrays.copyOf(result, values);
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

            linesArray = new int[0];
            try {
                linesArray = lines.toArray();
            }
            catch (Exception e) {
                telemetry.addData("No lines found", "");
            }

            cleanLines(5);
            telemetry.addData("Number of lines", linesArray.length / 4);

            for (int i = 0; i < linesArray.length; i += 4) {
                int x1 = linesArray[i], y1 = linesArray[i + 1], x2 = linesArray[i + 2], y2 = linesArray[i + 3];
                double length = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
                Imgproc.line(input, new Point(x1, y1), new Point(x2, y2), new Scalar(0, 0, 0), 1);
                Imgproc.putText(input, String.valueOf(Math.round(length)), new Point((x1 + x2) / 2.0, (y1 + y2) / 2.0), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 0), 1);
                telemetry.addData("Vertical Line Length", length);
            }
        }
        telemetry.update();
        return input;

    }



    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();

        //why this color format?
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2YCrCb);

        Mat yellowMask = new Mat();

        //why these values?
        Core.inRange(hsvFrame, new Scalar(0, 138, 0), new Scalar(255, 200, 100), yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

        //what does this do?
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
