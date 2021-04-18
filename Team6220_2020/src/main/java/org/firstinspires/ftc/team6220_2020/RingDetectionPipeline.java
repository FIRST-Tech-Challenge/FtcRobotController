package org.firstinspires.ftc.team6220_2020;

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

public class RingDetectionPipeline extends OpenCvPipeline {

    public static int ringStackHeight = 0;
    public static double ringPixelArea = 0;


    @Override
    public Mat processFrame(Mat input) {

        double maxArea;
        int maxAreaContour;

        Mat originalInput = input;

        //Todo cahnge the 50 to what ever to change the boarder
        Imgproc.rectangle(
                input,
                new Point(
                        0,
                        0),
                new Point(
                        input.cols(),
                        input.rows()),
                new Scalar(0, 0, 0), 25);

        Imgproc.rectangle(
                input,
                new Point(
                        0,
                        0),
                new Point(
                        input.cols(),
                        0),
                new Scalar(0, 0, 0), 250);

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2YUV);

        Imgproc.medianBlur(input, input, 11);

        List<Mat> yuvSplit = new ArrayList<>();
        Core.split(input, yuvSplit);

        Mat u = yuvSplit.get(1);

        Imgproc.threshold(u,u, 147,255, Imgproc.THRESH_BINARY);

        List<MatOfPoint> contours = new ArrayList<>();

        Mat hierarchy = new Mat();

        Imgproc.findContours(u, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if(contours.size() > 0){
            maxArea = 0.0;
            maxAreaContour = -1;

            for(int i = 0; i < contours.size(); i++){

                if(Imgproc.contourArea(contours.get(i)) > maxArea){
                    maxArea = Imgproc.contourArea(contours.get(i));
                    maxAreaContour = i;
                }

            }
            Imgproc.rectangle(originalInput, Imgproc.boundingRect(contours.get(maxAreaContour)), new Scalar(255, 0, 0));

            //todo modify the max and middle values.

            ringPixelArea = maxArea;

            if(maxArea > 2200/*Max size*/){
                ringStackHeight = 0;
            } else if(maxArea > 1200/*Middle size*/){
                ringStackHeight = 4;
            } else if(maxArea > 300){
                ringStackHeight = 1;
            } else{
                ringStackHeight = 0;
            }
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_YUV2BGR);
        return input;
    }
}
