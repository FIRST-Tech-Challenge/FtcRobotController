package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

public class DamienCVPipeline extends OpenCvPipeline {
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

        Imgproc.cvtColor(input,inputRGB,Imgproc.COLOR_RGBA2RGB);
//        Imgproc.cvtColor(input,grayInput,Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(inputRGB,hsvInput,Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvInput, new Scalar(30, 100, 100), new Scalar(80, 255, 255), inputGreen);
        Core.inRange(hsvInput, new Scalar(90, 100, 100), new Scalar(100, 255, 255), inputYellow);
        Core.inRange(hsvInput, new Scalar(150, 100, 100), new Scalar(190, 255, 255), inputPurple);

        purplePixels = Core.sumElems(inputPurple).val[0];
        greenPixels = Core.sumElems(inputGreen).val[0];
        yellowPixels = Core.sumElems(inputYellow).val[0];

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
                currentResultStr = "yellow";
                break;
            case 3:
                currentResultStr = "green";
                break;
        }
        return currentResultStr;
    }

}
