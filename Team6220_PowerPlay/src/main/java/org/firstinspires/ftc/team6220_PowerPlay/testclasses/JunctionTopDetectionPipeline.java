package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class JunctionTopDetectionPipeline extends OpenCvPipeline {
    double[] black = {60, 60, 60};
    double[] white = {235, 235, 235};

    private Mat blackMask = new Mat();

    public int framesRun = 0;


    @Override
    public Mat processFrame(Mat mat) {
        framesRun++;
        Core.inRange(mat, new Scalar(new double[]{0, 0, 0}), new Scalar(black), mat);

        return mat;
    }
}
