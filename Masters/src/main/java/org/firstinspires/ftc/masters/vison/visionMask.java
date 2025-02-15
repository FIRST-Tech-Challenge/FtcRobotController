package org.firstinspires.ftc.masters.vison;

import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class visionMask extends OpenCvPipeline {

    private final Scalar lower = new Scalar(110,150,20);
    private final Scalar upper = new Scalar(130,255,255);

    Mat HSV = new Mat();
    Mat H = new Mat();
    Mat S = new Mat();
    Mat V = new Mat();

    Mat mask = new Mat(), diff_im = new Mat();

    void inputToHSV(Mat input) {
        Imgproc.cvtColor(input,HSV,Imgproc.COLOR_BGR2HSV);
        Core.extractChannel(HSV, H, 0);
        Core.extractChannel(HSV, S, 1);
        Core.extractChannel(HSV, V, 2);
    }

    @Override
    public Mat processFrame(Mat input) {

        inputToHSV(input);

        Core.inRange(HSV, lower, upper, mask);

        diff_im = new Mat();
        Core.add(diff_im, Scalar.all(0), diff_im);
        //Core.bitwise_not(mask,mask);
        input.copyTo(diff_im, mask);
        diff_im.copyTo(input);
        diff_im.release();

        inputToHSV(input);

        return input;
    }
}