package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class pipeline3 extends OpenCvPipeline {
    private Mat mask1, mask2, hierarchy1 = new Mat(), hierarchy2 = new Mat();
    private Mat out = new Mat();

    public Scalar x = new Scalar(0,0,0);

    @Override
    public Mat processFrame(Mat input) {
        out = input;
        mask1 = new Mat(input.rows(), input.cols(), CvType.CV_8UC1);
        Core.inRange(input, new Scalar(160,90,10), new Scalar(255,160,40), mask1);

        mask2 = new Mat(input.rows(), input.cols(), CvType.CV_8UC1);
        Core.inRange(input, new Scalar(40,70,150), new Scalar(50, 90, 180), mask2);

        Imgproc.GaussianBlur(mask1, mask1, new Size(5.0, 15.0), 0.00);
        Imgproc.GaussianBlur(mask2, mask2, new Size(5.0, 15.0), 0.00);

        ArrayList<MatOfPoint> contours1 = new ArrayList<>(), contours2 = new ArrayList<>();

        Imgproc.findContours(mask1, contours1, hierarchy1, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(mask2, contours2, hierarchy2, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        Imgproc.drawContours(out,contours1,-1, new Scalar(255,0,0));
        Imgproc.drawContours(out, contours2, -1, new Scalar(0,255,0));

        return out;
    }
}
