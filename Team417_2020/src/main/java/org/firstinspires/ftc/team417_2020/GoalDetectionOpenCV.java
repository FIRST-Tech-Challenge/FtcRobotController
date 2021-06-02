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

    int sLower = 0;
    int sUpper = 10;
    int vLower = 100;
    int vUpper = 255;
    Scalar whiteLower = new Scalar(0, sLower, vLower);
    Scalar whiteUpper = new Scalar(255, sUpper, vUpper);
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
        //blue.release();
        whiteDetection(blueCropped);
        whiteGoal = whiteRect;


        int x1 = whiteGoal.x;
        int y1 = whiteGoal.y;
        int x2 = whiteGoal.width + x1;
        int y2 = whiteGoal.height + y1;
        Imgproc.rectangle(blueCropped, new Point(x1, y1), new Point(x2, y2),
                new Scalar(0, 0, 0), 3);

        return blueCropped;





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

        Imgproc.cvtColor(input, whiteHsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(whiteHsv, whiteLower, whiteUpper, whiteThreshold);
        Imgproc.findContours(whiteThreshold, whiteContours, whiteHierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0.0;
        // find largest contour
        int i = 0;
        for (MatOfPoint contour : whiteContours) {
            rect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            Imgproc.drawContours(input, whiteContours, i, blueColor);
            if (area > maxArea) {
                maxArea = area;
                whiteRect = rect;
            }
            i++;
        }
        whiteContours.clear();
        // put green rectangle around white area

        Imgproc.rectangle(input, new Point(whiteRect.x, whiteRect.y),
                new Point(whiteRect.x + whiteRect.width, whiteRect.y + whiteRect.height),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(input, new Point(49, 49), new Point(51, 51), new Scalar(0,0,0), 1);

        return input;
    }

    /**
     *
     * @param rect
     * @param mat
     * @return
     */
    public boolean rectIsInMiddleRangeOfMat(Rect rect, Mat mat) {
        boolean isInMiddleRange = true;

        if (rect.x < mat.rows() / 4.0) {
            isInMiddleRange = false;
        }
        if (rect.x + rect.width > (mat.rows() / 4.0) * 3) {
            isInMiddleRange = false;
        }
        return isInMiddleRange;
    }
}
