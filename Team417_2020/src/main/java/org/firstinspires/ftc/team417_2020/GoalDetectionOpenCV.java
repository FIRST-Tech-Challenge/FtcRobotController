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


// rbg
public class GoalDetectionOpenCV extends OpenCvPipeline {

    // process frame
    Rect whiteGoal = new Rect();
    boolean goalInView;
    double blueArea;
    Mat hsv = new Mat();
    Mat blue = new Mat();
    Mat displayMat = new Mat();
    Mat blueCropped = new Mat();


    // blue detection
    Mat blueGoal = new Mat();
    Rect blueRect = new Rect();
    Mat blueBlurred = new Mat();
    Mat blueThreshold = new Mat();
    List<MatOfPoint> blueContours = new ArrayList<>();
    Mat blueHierarchy = new Mat();
    Mat blueDisplayMat = new Mat();
    Rect rect = new Rect();
    Scalar blueLower = new Scalar(0,100,135); // 100, 135, 0     // 0,100,135
    Scalar blueUpper = new Scalar(254, 254, 254); // 254, 254, 254

    // white detection
    Mat whiteThreshold = new Mat();
    Rect whiteRect = new Rect();
    Mat whiteHierarchy = new Mat();
    List<MatOfPoint> whiteContours = new ArrayList<>();
    Mat whiteHsv = new Mat();
    Mat whiteCropped = new Mat();
    Scalar whiteLower = new Scalar(50, 0, 50);
    Scalar whiteUpper = new Scalar(80, 0, 80);
    Mat r = new Mat();
    Mat g = new Mat();
    Mat b = new Mat();

    Rect croppingRect = new Rect(110, 50, 15, 15);
    Scalar blueColor = new Scalar(0, 0, 255);
    //double[] values1 = new double[2];



    @Override
    public Mat processFrame(Mat input) {
        //return input;
        input.copyTo(blue);
        input.copyTo(displayMat);

        blueCropped = blueDetection(blue);
        blue.release();
        /*whiteGoal = whiteDetection(blue);


        int x1 = whiteGoal.x;
        int y1 = whiteGoal.y;
        int x2 = whiteGoal.width + x1;
        int y2 = whiteGoal.height + y1;
        Imgproc.rectangle(displayMat, new Point(x1, y1), new Point(x2, y2),
                new Scalar(0, 0, 0), 3);
        Imgproc.rectangle(displayMat, new Point(blueRect.x, blueRect.y),
                new Point(blueRect.x + blueRect.width, blueRect.y + blueRect.height),
                new Scalar(0, 0, 255), 1);
        return displayMat;*/

        return whiteDetection(blueCropped);



    }

    public Mat blueDetection(Mat input) {
        input.copyTo(blueDisplayMat);
        Imgproc.medianBlur(input, blueBlurred, 15);
        Imgproc.cvtColor(blueBlurred, hsv, Imgproc.COLOR_BGR2HSV);



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

        //Imgproc.cvtColor(input, whiteHsv, Imgproc.COLOR_BGR2HSV);


        Core.inRange(input, whiteLower, whiteUpper, whiteThreshold);
        Imgproc.findContours(whiteThreshold, whiteContours, whiteHierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0.0;
        // find largest contour
        for (MatOfPoint contour : whiteContours) {
            rect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                whiteRect = rect;
            }
        }
        // put green rectangle around white area
        Imgproc.rectangle(input, new Point(whiteRect.x, whiteRect.y),
                new Point(whiteRect.x + whiteRect.width, whiteRect.y + whiteRect.height),
                new Scalar(0, 255, 0), 3);
        // put blue rectangle around image roi
        /*Imgproc.rectangle(input, new Point(110, 50), new Point(125, 65),
                new Scalar(0, 0, 255), 2);


        if (input.size().height * input.size().width > 7200) {
            values1 = input.get(124, 64);
            // (120, 60): 62, 0, 61

            Imgproc.putText(input, values1[0] + ", " + values1[1] + ", " + values1[2], new Point(50, 20),
                    0, 1, blueColor);
        }*/
        /*double[] values2 = input.get(122, 62);
        double[] values3 = input.get(124, 64);
        double[] values4 = input.get(118, 58);
        double[] values5 = input.get(116, 56);*/

        //Imgproc.putText(input, values1.toString(), new Point(50, 20),
              //  0, 1, blueColor);
        /*Imgproc.putText(input, values2.toString(), new Point(50, 30),
                0, 1, blueColor);
        Imgproc.putText(input, values3.toString(), new Point(50, 40),
                0, 1, blueColor);
        Imgproc.putText(input, values4.toString(), new Point(50, 50),
                0, 1, blueColor);
        Imgproc.putText(input, values5.toString(), new Point(50, 60),
                0, 1, blueColor);*/


        //whiteCropped = input.submat(croppingRect);
        /*Core.extractChannel(whiteCropped, r, 0);
        Core.extractChannel(whiteCropped, b, 1);
        Core.extractChannel(whiteCropped, g, 2);
        double rMean = Core.mean(r).val[0];
        double bMean = Core.mean(b).val[0];
        double gMean = Core.mean(g).val[0];

        Imgproc.putText(input, "R mean: " + rMean, new Point(50, 40),0, 1, new Scalar(255,0,0));
        Imgproc.putText(input, "B mean: " + bMean, new Point(50, 50),0, 1, new Scalar(255,0,0));
        Imgproc.putText(input, "G mean: " + gMean, new Point(50, 60),0, 1, new Scalar(255,0,0));*/

        //return input;
        return input;
    }
}
