package org.firstinspires.ftc.team6220_2021;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BarcodeDetectionPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        double maximumArea;
        int maximumAreaContour;

        Imgproc.rectangle(input, new Point(0, 0), new Point(input.cols(), input.rows()), new Scalar(0, 0, 0), 25);

        Imgproc.rectangle(input, new Point(0, 0), new Point(input.cols(), 0), new Scalar(0, 0, 0), 0);

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);

        Imgproc.medianBlur(input, input, 10);

        List<Mat> hsv = new ArrayList<>();

        Core.split(input, hsv);

        Mat v = hsv.get(2);

        Imgproc.threshold(v,v, 200, 255, Imgproc.THRESH_BINARY);

        List<MatOfPoint> contours = new ArrayList<>();

        Mat hierarchy = new Mat();

        Imgproc.findContours(v, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            maximumArea = 0.0;
            maximumAreaContour = -1;

            for (int i = 0; i < contours.size(); i++) {
                if (Imgproc.contourArea(contours.get(i)) > maximumArea) {
                    maximumArea = Imgproc.contourArea(contours.get(i));
                    maximumAreaContour = i;
                }
            }

            Imgproc.rectangle(input, Imgproc.boundingRect(contours.get(maximumAreaContour)), new Scalar(255, 0, 0));

            if (maxArea > 2200/*Max size*/){
                ringStackHeight = 0;
            } else if(maxArea > 1200/*Middle size*/){
                ringStackHeight = 4;
            } else if(maxArea > 300){
                ringStackHeight = 1;
            } else{
                ringStackHeight = 0;
            }
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2BGR);
        return input;
    }
}