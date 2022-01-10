package org.firstinspires.ftc.teamcode.calibration;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class singleCalib extends OpenCvPipeline {
    private Mat x;

    @Override
    public Mat processFrame(Mat input) {
        x = input;
        return null;
    }

    public Mat getMat() {
        Imgproc.cvtColor(x, x, Imgproc.COLOR_BGR2GRAY);
        return x;


    }
}
