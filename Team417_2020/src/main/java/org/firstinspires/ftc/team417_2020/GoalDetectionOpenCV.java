package org.firstinspires.ftc.team417_2020;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class GoalDetectionOpenCV extends OpenCvPipeline {

    // process frame
    Rect whiteGoal = new Rect();
    boolean goalInView;
    double blueArea;
    Mat hsv = new Mat();
    Mat blue = new Mat();

    // blue detection
    Mat blueGoal = new Mat();
    Rect blueRect = new Rect();
    Mat blueBlurred = new Mat();
    Mat blueThreshold = new Mat();
    List<MatOfPoint> blueContours = new ArrayList<>();
    Mat blueHierarchy = new Mat();
    Mat blueDisplayMat = new Mat();
    Rect rect = new Rect();

    // white detection
    Mat whiteMask = new Mat();
    Mat whiteThreshold = new Mat();
    List<MatOfPoint> whiteContours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        //return input;
        input.copyTo(blue);

        return blueDetection(blue);

    }

    public Mat blueDetection(Mat input) {
        input.copyTo(blueDisplayMat);
        Imgproc.medianBlur(input, blueBlurred, 15);
        Imgproc.cvtColor(blueBlurred, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar blueLower = new Scalar(0, 100, 135); // 100, 135, 0
        Scalar blueUpper = new Scalar(254, 254, 254); // 254, 254, 254

        Core.inRange(hsv, blueLower, blueUpper, blueThreshold);
        Imgproc.findContours(blueThreshold, blueContours, blueHierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);

        // find largest contour
        double maxArea = 0.0;
        for (MatOfPoint contour : blueContours) {
            rect = Imgproc.boundingRect(contour);

            double area = Imgproc.contourArea(contour);
            if (area > maxArea){
                maxArea = area;
                blueRect = rect;
            }
        }
        blueContours.clear();

        Imgproc.rectangle(blueDisplayMat,
                    new Point(blueRect.x, blueRect.y),
                    new Point(blueRect.x + blueRect.width, blueRect.y + blueRect.height),
                    new Scalar(0,254,0), 10);
        blueGoal = input.submat(blueRect);

        return blueGoal;

    }

    // todo test white filtering parameters on field
    public Mat whiteDetection(Mat input) {

        Scalar whiteLower = new Scalar(200, 200, 200);
        Scalar whiteUpper = new Scalar(255, 255, 255);

        Core.inRange(input, whiteLower, whiteUpper, whiteThreshold);
        Imgproc.findContours(blueThreshold, blueContours, blueHierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0.0;
        // find largest contour
        for (MatOfPoint contour : whiteContours) {
            rect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                
            }
        }
        return whiteMask;
    }
}
