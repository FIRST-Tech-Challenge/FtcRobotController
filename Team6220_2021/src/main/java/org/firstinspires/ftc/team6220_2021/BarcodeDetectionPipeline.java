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

    static int barcode;

    public double contourAreaFinder(Mat input) {
        Imgproc.medianBlur(input, input, 11);

        List<Mat> bgrSplit = new ArrayList<>();

        Core.split(input, bgrSplit);

        Mat r = bgrSplit.get(2);

        Imgproc.threshold(r,r, 150, 255, Imgproc.THRESH_BINARY_INV);

        List<MatOfPoint> contours = new ArrayList<>();

        Mat hierarchy = new Mat();

        Imgproc.findContours(r, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            maximumArea = 0.0;
            int maximumAreaContour = -1;

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

        Mat left = input.submat(leftBarcode);
        Mat center = input.submat(centerBarcode);
        Mat right = input.submat(rightBarcode);

        leftArea = contourAreaFinder(left);
        centerArea = contourAreaFinder(center);
        rightArea = contourAreaFinder(right);

        if (leftArea > centerArea && leftArea > rightArea) {
            barcode = 1;
        }
        else if (centerArea > leftArea && centerArea > rightArea) {
            barcode = 2;
        }
        else if (rightArea > leftArea && rightArea > centerArea) {
            barcode = 3;
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2BGR);
        return input;
    }
}