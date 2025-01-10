package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        Scalar lower_blue = new Scalar(85,50, 40);
        Scalar higher_blue = new Scalar(135, 255, 255);

        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Mat threshold = new Mat();
        Core.inRange(hsvMat, lower_blue, higher_blue, threshold);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierachy = new Mat();

        Imgproc.findContours(threshold, contours, hierachy,Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 3);

        return null;
    }
}
