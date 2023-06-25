package org.firstinspires.ftc.teamcode.commandBased.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {

    @Override
    public void init(Mat input) {
        // Executed before the first call to processFrame
    }

    @Override
    public Mat processFrame(Mat input) {

        //create mats
        Mat inputMat = input;
        Mat HSVMat = new Mat();
        Mat threshMat = new Mat();
        Mat maskedMat = new Mat();
        Mat scaledMat = new Mat();
        Mat scaledThreshMat = new Mat();
        Mat outputMat = new Mat();
        Mat edgesMat = new Mat();

        //convert things to RGB/HSV
        Imgproc.cvtColor(input, inputMat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input, HSVMat, Imgproc.COLOR_RGB2HSV);

        //apply lenient HSV filter
        Scalar lowHSV = new Scalar(86, 44, 23);
        Scalar highHSV = new Scalar(128, 255, 249);
        Core.inRange(HSVMat, lowHSV, highHSV, threshMat);

        //return blue values
        Core.bitwise_and(inputMat, inputMat, maskedMat, threshMat);

        //scale avg saturation to 150
        Scalar average = Core.mean(maskedMat, threshMat);
        maskedMat.convertTo(scaledMat, -1, 150/average.val[1], 0);

        //stricter HSV
        Scalar strictLowHSV = new Scalar(0, 25, 100);
        Scalar strictHigherHSV = new Scalar(100, 175, 255);

        Core.inRange(scaledMat, strictLowHSV, strictHigherHSV, scaledThreshMat);

        Core.bitwise_and(inputMat, HSVMat, outputMat, scaledThreshMat);

        Imgproc.Canny(outputMat, edgesMat, 100, 200);

        return scaledThreshMat;
    }

    @Override
    public void onViewportTapped() {
        // Executed when the image display is clicked by the mouse or tapped
        // This method is executed from the UI thread, so be careful to not
        // perform any sort heavy processing here! Your app might hang otherwise
    }

}