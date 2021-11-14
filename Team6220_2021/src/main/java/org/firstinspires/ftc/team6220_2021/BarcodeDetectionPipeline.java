package org.firstinspires.ftc.team6220_2021;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BarcodeDetectionPipeline extends OpenCvPipeline {

    double maximumArea;
    double leftArea;
    double centerArea;
    double rightArea;
    int maximumAreaContour;
    static String position;

    public double contourArea(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2GRAY);

        Imgproc.medianBlur(input, input, 5);

        List<Mat> split = new ArrayList<>();

        Core.split(input, split);

        Mat x = split.get(1);

        Imgproc.threshold(x, x, 200, 255, Imgproc.THRESH_BINARY);

        List<MatOfPoint> contours = new ArrayList<>();

        Mat hierarchy = new Mat();

        Imgproc.findContours(x, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            maximumArea = 0.0;
            maximumAreaContour = -1;

            for (int i = 0; i < contours.size(); i++) {
                if (Imgproc.contourArea(contours.get(i)) > maximumArea) {
                    maximumArea = Imgproc.contourArea(contours.get(i));
                    maximumAreaContour = i;
                }
            }
        }

        return maximumArea;
    }

    public Mat processFrame(Mat input) {
        Rect leftBarcode = new Rect(0, 0, 200, 500);
        Rect centerBarcode = new Rect(200, 0, 200, 500);
        Rect rightBarcode = new Rect(400, 0, 200, 500);

        Mat left = new Mat(input, leftBarcode);
        Mat center = new Mat(input, centerBarcode);
        Mat right = new Mat(input, rightBarcode);

        leftArea = contourArea(left);
        centerArea = contourArea(center);
        rightArea = contourArea(right);

        if (leftArea > centerArea && leftArea > rightArea) {
            position = "left";
        }
        else if (centerArea > leftArea && centerArea > rightArea){
            position = "center";
        }
        else if (rightArea > leftArea && rightArea > centerArea){
            position = "right";
        }
        else {
            position = "none";
        }

        return input;
    }
}