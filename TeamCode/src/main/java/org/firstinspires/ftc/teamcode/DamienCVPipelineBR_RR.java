package org.firstinspires.ftc.teamcode;

import com.vuforia.Rectangle;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

public class DamienCVPipelineBR_RR extends OpenCvPipeline {
    int lastResult = 0;
    String currentResultStr = "";
    double greenPixels = 0;
    double purplePixels = 0;
    double yellowPixels = 0;
    int currentView = 0;
    double[] pixelNumArrays;
    Mat inputRGB = new Mat();
    Mat hsvInput = new Mat();
    Mat inputGreen = new Mat();
    Mat inputYellow = new Mat();
    Mat inputPurple = new Mat();
    Mat returnedMat = new Mat();



    @Override
    public Mat processFrame(Mat input) {
//        Mat grayInput = new Mat();

        Imgproc.cvtColor(input,inputRGB,Imgproc.COLOR_BGR2RGB);
//        Imgproc.cvtColor(input,grayInput,Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(inputRGB,hsvInput,Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvInput, new Scalar(28, 46, 51), new Scalar(68, 240, 255), inputGreen); // 30 - 80 100 - 255 s 100-255 v
        Core.inRange(hsvInput, new Scalar(81, 115, 164), new Scalar(107, 255, 255), inputYellow); // 90 - 100 h
        Core.inRange(hsvInput, new Scalar(159, 46, 51), new Scalar(184, 255, 255), inputPurple); // 150 - 190 h

        Rect LEFT_ROI = new Rect(new Point(0,0), new Point(600,720));

        Mat subPurple = inputPurple.submat(LEFT_ROI);
        Mat subGreen = inputGreen.submat(LEFT_ROI);
        Mat subYellow = inputYellow.submat(LEFT_ROI);

        purplePixels = Core.sumElems(subPurple).val[0];
        greenPixels = Core.sumElems(subGreen).val[0];
        yellowPixels = Core.sumElems(subYellow).val[0];

        pixelNumArrays = new double[]{purplePixels, greenPixels, yellowPixels};
        Arrays.sort(pixelNumArrays);
        if (pixelNumArrays[2] == purplePixels) {
            lastResult = 1;
        }   if(pixelNumArrays[2] == greenPixels) {
            lastResult = 2;
        }   if (pixelNumArrays[2] == yellowPixels) {
            lastResult = 3;
        }
        if(currentView == 0) {
            returnedMat = input;
        } if(currentView == 1) {
            returnedMat = inputPurple;
        } if(currentView == 2) {
            returnedMat = inputYellow;
        } if(currentView == 3) {
            returnedMat = inputGreen;
        }

        switch (lastResult) {
            case 1:
                currentResultStr = "purple";
                break;
            case 2:
                currentResultStr = "green";
                break;
            case 3:
                currentResultStr = "yellow";
                break;
        }
//        input.release();
//        inputRGB.release();
//        inputGreen.release();
//        inputPurple.release();
//        inputYellow.release();
//        hsvInput.release();
//        subGreen.release();
//        subPurple.release();
//        subYellow.release();
        return returnedMat;
    }
    public int getCurrentResults() {
        return lastResult;
    }
    public String getCurrentResultsStr() {
        switch (lastResult) {
            case 1:
                currentResultStr = "purple";
                break;
            case 2:
                currentResultStr = "green";
                break;
            case 3:
                currentResultStr = "yellow";
                break;
        }
        return currentResultStr;
    }

}
