package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Size;

public class GrabberCameraPipeline extends OpenCvPipeline {
    double[] black = {200, 200, 200};
    double[] white = {235, 235, 235};

    public int junctionX = 0;
    public int junctionY = 0;

    private Mat blackMask = new Mat();

    @Override
    public Mat processFrame(Mat mat) {
        Size blurSize = new Size(5,5);
        Mat hsv = new Mat();
        Imgproc.cvtColor(mat,hsv,Imgproc.COLOR_RGB2HSV);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.blur(hsv,hsv, blurSize);
        Core.inRange(hsv, new Scalar(0, 0, 0), new Scalar(255, 255, 30), blackMask);
        Mat hierarchy = new Mat();
        Imgproc.findContours(blackMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 0) {
            double maxVal = 0.0;
            int maxValIdx = 0;

            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));

                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }

            // draw bounding rectangle around the largest contour
            Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));
            Imgproc.rectangle(mat, boundingRect, new Scalar(40, 200, 0), 10);
            junctionX = boundingRect.x + (boundingRect.width / 2);
            junctionY = boundingRect.y + (boundingRect.height / 2);
        }

        return mat;
    }
}
