package org.firstinspires.ftc.teamcode.VisionPipelines;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.*;
public class poleDetectionPipeline extends OpenCvPipeline {
    Mat gray = new Mat();
    Mat edgeDetectorFrame = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    public double bottomThreshold = 100;
    public double topThreshold = 300;
    Scalar blue = new Scalar(0, 0, 0);

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        // Order: input image, output edges(Array), lowerthreshold, upperthreshold
        Imgproc.Canny(gray, edgeDetectorFrame, bottomThreshold, topThreshold);
        contours.clear();
        Imgproc.findContours(edgeDetectorFrame, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Order : input image, the contours from output, hierarchy, mode, and method
        Imgproc.drawContours(input, contours, -1, blue);
        return input;
    }
}


/*
    Mat gray = new Mat();
    Mat edgeDetectorFrame = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    double bottomThreshold = 100;
    double topThreshold = 300;
    Scalar blue = new Scalar(0, 0, 255);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.Canny(gray, edgeDetectorFrame, bottomThreshold, topThreshold);
        contours.clear();
        Imgproc.findContours(edgeDetectorFrame, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, blue);
        return input;
    }
 */