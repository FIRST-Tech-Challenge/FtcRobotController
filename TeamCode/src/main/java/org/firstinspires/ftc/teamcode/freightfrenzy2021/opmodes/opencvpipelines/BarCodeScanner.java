package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

public class BarCodeScanner extends OpenCvPipeline {
    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat leftHsv = new Mat();
    Mat rightHsv = new Mat();
    Mat leftTargetMat;
    Mat rightTargetMat;
    boolean firstPass = true;
    int readingCount = 0;
    boolean readingConsumed = false;    // manage state of whether reading has been consumed
    Rect leftRect = new Rect(new Point(10, 170), new Point(80, 200));
    Rect rightRect = new Rect(new Point(165, 170), new Point(230, 200));

    float leftConfidenceRed = 0.0f;
    float rightConfidenceRed = 0.0f;

    public boolean isReadingConsumed() {
        return readingConsumed;
    }

    public int getReadingCount() {
        return readingCount;
    }

    public float getLeftConfidenceRed() {
        return leftConfidenceRed;
    }

    public float getRightConfidenceRed() {
        return rightConfidenceRed;
    }

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Static Methods
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // No static methods defined


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public void init(Mat firstFrame) {
        Log.d("EBOTS", "Mat size: " + firstFrame.size().toString());
        leftTargetMat = firstFrame.submat(leftRect);
        rightTargetMat = firstFrame.submat(rightRect);
        Log.d("EBOTS", "SubMat size: " + leftTargetMat.size().toString());
        Log.d("EBOTS", "First cell values " + Arrays.toString(leftTargetMat.get(0,0)));
    }

    @Override
    public Mat processFrame(Mat mat) {
        Imgproc.cvtColor(leftTargetMat, leftHsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(rightTargetMat, rightHsv, Imgproc.COLOR_RGB2HSV);
        leftConfidenceRed = calculateConfidenceRed(leftHsv);
        rightConfidenceRed = calculateConfidenceRed(rightHsv);
        readingConsumed = false;    // flat the current value as a new reading
        readingCount++;

        if (firstPass) {
            Log.d("EBOTS", "hsv size: " + leftHsv.size().toString());
            Log.d("EBOTS", "Lefthsv cols: " + String.format("%d", leftHsv.cols()));
            Log.d("EBOTS", "val1: " + Arrays.toString(leftHsv.get(0,0)));
            Log.d("EBOTS", "val2: " + Arrays.toString(leftHsv.get(14,34)));
            Log.d("EBOTS", "val3: " + Arrays.toString(leftHsv.get(29,69)));
            Log.d("EBOTS", "Right hsv cols: " + String.format("%d", rightHsv.cols()));
            Log.d("EBOTS", "val1: " + Arrays.toString(rightHsv.get(0,0)));
            Log.d("EBOTS", "val2: " + Arrays.toString(rightHsv.get(14,34)));
            Log.d("EBOTS", "val3: " + Arrays.toString(rightHsv.get(29,64)));
            Log.d("EBOTS", "Left Confidence Red: " + String.format("%.1f", leftConfidenceRed));
            Log.d("EBOTS", "Right Confidence Red: " + String.format("%.1f", rightConfidenceRed));
        }

        // draw a bounding rectangle
        Scalar rectColor = new Scalar(255,10,10);
        int thickness = 2;
        Imgproc.rectangle (mat,leftRect, rectColor, thickness);
        Imgproc.rectangle (mat,rightRect, rectColor, thickness);

        firstPass = false;

        return mat;
    }



    public void markReadingAsConsumed(){
        readingConsumed = true;
    }

    @Deprecated
    private int calculateAverageHue(Mat hsv){
        // Average Caused issues with red readings, which can be 150-180 and 0-40 in low light
        int sum = 0;

        int divisor = hsv.rows() * hsv.cols();
        for (int row=0; row < hsv.rows(); row++){
            for (int col=0; col < hsv.cols(); col++){
                sum += hsv.get(row, col)[0];
            }
        }

        double average = ((double) sum) / divisor;
        return (int) average;

    }

    private float calculateConfidenceRed(Mat hsv){
        int numRed = 0;
        int totalPixels = hsv.rows() * hsv.cols();
        double valueThreshold = 50.0;
        int validPixels = 0;
        double pixelValue = 0;
//        double hueMin = 200.0;
//        double hueMax = 0.0;
        for (int row=0; row < hsv.rows(); row++){
            for (int col=0; col < hsv.cols(); col++){

                pixelValue = hsv.get(row,col)[2];
                if (pixelValue > valueThreshold) {
                    validPixels++;
                    // if value is high enough (ignores black)
                    double currentHue = hsv.get(row, col)[0];
                    if (currentHue < 30 | currentHue > 150) {
                        numRed++;
                    }
                } else{
                    // exclude it from analysis
                }
//                hueMin = Math.min(hueMin, currentHue);
//                hueMax = Math.max(hueMax, currentHue);
            }
        }
//        String logTag = "EBOTS";
//        String oneDec = "%.1f";
//        Log.d(logTag, "MIN -- Max:  " + String.format(oneDec, hueMin) + " -- " +
//                String.format(oneDec, hueMax));
//        Log.d(logTag, "Num Red / divisor: " + String.format("%d", numRed) +
//                " / " + String.format("%d", divisor));
//        Log.d(logTag, "Percentage valid pixels: " + String.format("%.2f", ((float) validPixels) / totalPixels));

        float percentValidPixels = ((float) validPixels) / totalPixels;
        // Need to have at least 40% valid pixels
        float redPercentage = (percentValidPixels > 0.40) ? ((float)numRed / validPixels) : 0.0f;
//        Log.d(logTag, "% Red" + String.format("%.2f", redPercentage));
        return  redPercentage;
    }
}
