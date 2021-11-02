package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RGB2YCbCrMask extends OpenCvPipeline {
    public Scalar low = new Scalar(0,0,0,0), high = new Scalar(255,255,255,255);
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);
        return input;
    }
}
