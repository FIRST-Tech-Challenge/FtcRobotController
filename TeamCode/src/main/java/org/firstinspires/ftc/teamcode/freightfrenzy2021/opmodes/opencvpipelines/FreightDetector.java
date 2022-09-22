package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines;

import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

public class FreightDetector extends OpenCvPipeline {
    Mat frameHsv = new Mat();
    Mat frameTargetMat;
    boolean firstPass = true;
    int readingCount = 0;
    boolean readingConsumed = false;
    String logTag = "EBOTS";
    boolean isBox = false;
    boolean isBall = false;
    double confidenceBox;
    double confidenceBall;
    boolean hsvBoxDebug = false;
    boolean hsvBallDebug = false;

    //Check the co ordinates
//    Rect frameRect = new Rect(new Point(130, 20), new Point(190, 80));
    Rect frameRect = new Rect(new Point(130, 50), new Point(190, 80));
    int averageHue = 0;
    private boolean wasBall = false;
    private boolean wasBox = false;

    public boolean isReadingConsumed() {
        return readingConsumed;
    }

    public int getReadingCount() {
        return readingCount;
    }

    public float getAverageHue() {
        return averageHue;
    }

    public boolean getIsBox() {
        return isBox;
    }

    public boolean getIsBall() {
        return isBall;
    }

    public double getConfidenceBox() {
        return confidenceBox;
    }

    public double getConfidenceBall() {
        return confidenceBall;
    }

    @Override
    public void init(Mat firstFrame) {
        Log.d("EBOTS", "Mat size: " + firstFrame.size().toString());
        frameTargetMat = firstFrame.submat(frameRect);
        Log.d("EBOTS", "SubMat size: " + frameTargetMat.size().toString());
    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(frameTargetMat, frameHsv, Imgproc.COLOR_RGB2HSV);
        averageHue = calculateAverageHue(frameHsv);
        determineFreightPresence();

        if (firstPass) {
            Log.d("EBOTS", "hsv size: " + frameHsv.size().toString());
            Log.d("EBOTS", "hsv cols: " + String.format("%d", frameHsv.cols()));
            Log.d("EBOTS", "val1: " + Arrays.toString(frameHsv.get(0,0)));
//            Log.d("EBOTS", "val2: " + Arrays.toString(frameHsv.get((input.rows()/2),(input.cols()/2))));
            Log.d("EBOTS", "val2: " + Arrays.toString(frameHsv.get(14,29)));
//            Log.d("EBOTS", "val3: " + Arrays.toString(frameHsv.get(input.cols()-1,input.rows()-1)));
            Log.d("EBOTS", "val3: " + Arrays.toString(frameHsv.get(29,59)));
            Log.d("EBOTS", "Average Hue: " + String.format("%d", averageHue));
        }
        // draw a bounding rectangle
        Scalar rectColor = new Scalar(255,10,10);
        int thickness = 2;
        Imgproc.rectangle (input,frameRect, rectColor, thickness);
        firstPass = false;
        return input;
    }

    private void determineFreightPresence() {
        // calculate values for this loop
        double confidenceThresholdBox = 0.7;
        double confidenceThresholdBall = 0.50;
        confidenceBox = calculateConfidenceBox(frameHsv);
        confidenceBall = calculateConfidenceBall(frameHsv);
        boolean nowBox = confidenceBox >= confidenceThresholdBox;
        boolean nowBall = confidenceBall >= confidenceThresholdBall;

        // set freight conditions
        isBox = wasBox && nowBox;
        isBall = wasBall && nowBall;

        // set the wasBall and wasBox conditions for next loop
        wasBox = nowBox;
        wasBall = nowBall;

        readingConsumed = false;    // flat the current value as a new reading
        readingCount++;
    }


    private double calculateConfidenceBox(Mat hsv){
        int pixelCount = 0;
        double pixelHue;
        double pixelSaturation;
        double pixelValue;

        int boxCount=0;
        for (int row = 0; row < hsv.rows(); row++) {
            for (int col = 0; col < hsv.cols(); col++) {
                pixelHue = hsv.get(row, col)[0];
                pixelSaturation = hsv.get(row, col)[1];
                pixelValue = hsv.get(row, col)[2];
                pixelCount++;
                // if value is high enough (ignores black)
                boolean hueFlag = (pixelHue >= 0.0 && pixelHue <= 40.0) | pixelHue >= 160.0;
                boolean saturationFlag = pixelSaturation >= 85;
                //boolean valueFlag = pixelValue >= 160.0;
                boolean valueFlag = pixelValue >= 60.0;
                if (hueFlag && saturationFlag && valueFlag) {
                    boxCount++;
                } else{
                    if (hsvBoxDebug) Log.d(logTag, "Negative: " + Arrays.toString(hsv.get(row, col)));
                }
            }
        }
        double cubeConfidence = ((double) boxCount) / pixelCount;
//        Log.d(logTag, "boxCount / pixelCount: " + String.format("%.2f", cubeConfidence));
        return cubeConfidence;
    }

    private double calculateConfidenceBall(Mat hsv){
        int pixelCount = 0;
        double pixelHue;
        double pixelSaturation;
        double pixelValue;

        int ballCount=0;
        for (int row = 0; row < hsv.rows(); row++) {
            for (int col = 0; col < hsv.cols(); col++) {
                pixelHue = hsv.get(row, col)[0];
                pixelSaturation = hsv.get(row, col)[1];
                pixelValue = hsv.get(row, col)[2];
                pixelCount++;
                boolean hueFlag = pixelHue >= 90.0 && pixelHue <= 170;
                boolean saturationFlag = pixelSaturation >= 0 && pixelSaturation <=150;
                boolean valueFlag = pixelValue > 110.0;
                // if value is high enough (ignores black)
                if (hueFlag && saturationFlag && valueFlag) {
                    ballCount++;
                } else{
                    if (hsvBallDebug) Log.d(logTag, "Negative: " + Arrays.toString(hsv.get(row, col)));
                }
            }
        }
        double ballConfidence = ((double) ballCount) / pixelCount;
//        Log.d(logTag, "ballCount / pixelCount: " + String.format("%.2f", ballConfidence));
        return ballConfidence;
    }

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
        averageHue = (int) average;

//        logColorValues(hsv);

        return (int) average;
    }

    public void markReadingAsConsumed(){
        readingConsumed = true;
    }

    public void logColorValues(Mat hsv){
        Log.d("EBOTS", "val1: " + Arrays.toString(hsv.get(0,0)));
        Log.d("EBOTS", "val2: " + Arrays.toString(hsv.get(hsv.rows()/2,hsv.cols()/2)));
        Log.d("EBOTS", "val3: " + Arrays.toString(hsv.get(hsv.rows()-1,hsv.cols()-1)));
        Log.d("EBOTS", "Avg Hue: " + String.format("%d", averageHue));

    }
}
