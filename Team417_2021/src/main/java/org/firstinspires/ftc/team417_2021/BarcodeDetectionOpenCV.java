package org.firstinspires.ftc.team417_2021;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BarcodeDetectionOpenCV extends OpenCvPipeline {

    Mat frame = new Mat();
    Mat yuvFrame = new Mat();
    Mat yuv = new Mat();
    Mat yuvLeft = new Mat();
    Mat yuvRight = new Mat();
    Mat yuvMiddle = new Mat();
    Mat maskLeft = new Mat();
    Mat maskMiddle = new Mat();
    Mat maskRight = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();
    Mat blurred = new Mat();

    double maxArea = Double.MIN_VALUE;
    double leftMaxArea = 0;
    double rightMaxArea = 0;
    double middleMaxArea = 0;
    int barcodeIndex = 0;

    List<MatOfPoint> leftContours = new ArrayList<>();
    List<MatOfPoint> middleContours = new ArrayList<>();
    List<MatOfPoint> rightContours = new ArrayList<>();
    List<MatOfPoint> contours = new ArrayList<>();
    List<MatOfPoint> mainContours = new ArrayList<>();

    //                         y, u, v
    Scalar lower = new Scalar (0, 62, 0);
    Scalar higher = new Scalar (255, 117, 116);
    Scalar blackScalar = new Scalar (0,0,0);

    Rect rect = new Rect();
    Rect maxRect = new Rect();

    int x;
    int index;


    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(frame);


        Imgproc.cvtColor(input, yuv, Imgproc.COLOR_RGB2YUV);/*
        yuvLeft = yuv.submat(160,350, 0, 213);
        yuvMiddle = yuv.submat(160, 350, 213, 426);
        yuvRight = yuv.submat(160, 350, 426, 640);

        leftContours = getGreenContours(maskLeft);
        middleContours = getGreenContours(maskMiddle);
        rightContours = getGreenContours(maskRight);

        int i = 0;
        mainContours = getGreenContours(yuv);
        for (MatOfPoint contour : mainContours) {
            Imgproc.drawContours(frame, mainContours, i, blackScalar, 5);
            i++;
        }*/
        //Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        /*
        leftMaxArea = getLargestContourArea(leftContours);
        middleMaxArea = getLargestContourArea(middleContours);
        rightMaxArea = getLargestContourArea(rightContours);
        */

        //findBarcodeIndex();
        maxArea = 0.0;

        Imgproc.GaussianBlur(yuv, blurred, new Size(13,13), 0);
        Core.inRange(blurred, lower, higher, mask);

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            rect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                // max rect is not changing
                maxRect = rect;
                maxArea = area;
                x = maxRect.x;
            }
        }
        contours.clear();


/*
        maskLeft = mask.submat(160,350, 0, 213);
        maskMiddle = mask.submat(160, 350, 213, 426);
        maskRight = mask.submat(160, 350, 426, 640);

        leftMaxArea = tempMethodName(maskLeft);
        middleMaxArea = tempMethodName(maskMiddle);
        rightMaxArea = tempMethodName(maskRight);*/

        index = findBarcodeIndex2(x);

        return input;
    }

    public double tempMethodName(Mat input) {
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
            }
        }
        maxArea = 0;
        contours.clear();
        return maxArea;

    }

    public double getLargestContourArea(List<MatOfPoint> contours) {
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
            }
        }
        maxArea = 0;
        contours.clear();
        return maxArea;
    }

    public List<MatOfPoint> getGreenContours(Mat input) {
        Imgproc.GaussianBlur(input, blurred, new Size(13,13), 0);
        Core.inRange(blurred, lower, higher, mask);
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);

        return contours;
    }

    public int findBarcodeIndex() {
        if (leftMaxArea >= middleMaxArea && leftMaxArea >= rightMaxArea) {
            barcodeIndex = 0;
        }
        else if (middleMaxArea >= leftMaxArea && middleMaxArea >= rightMaxArea) {
            barcodeIndex = 1;
        }
        else if (rightMaxArea >= leftMaxArea && rightMaxArea >= middleMaxArea) {
            barcodeIndex = 2;
        }
        return barcodeIndex;
    }

    public int findBarcodeIndex2(int x) {
        // 213, 426
        int index = 0;
        if (x < 163) {
            index = 0;
        } else if (x > 163 && x < 376) {
            index = 1;
        } else if (x > 376) {
            index = 2;
        }
        return index;
    }

}
