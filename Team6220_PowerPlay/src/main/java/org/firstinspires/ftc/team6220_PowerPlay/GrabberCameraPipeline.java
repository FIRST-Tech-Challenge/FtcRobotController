package org.firstinspires.ftc.team6220_PowerPlay;

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
    public double xPosition = 0.0;
    public double yPosition = 0.0;

    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    public Mat hsv = new Mat();
    public Mat blackMask = new Mat();
    public Size blurSize = new Size(5,5);

    @Override
    public Mat processFrame(Mat mat) {
        Imgproc.cvtColor(mat,hsv,Imgproc.COLOR_RGB2HSV);

        Imgproc.blur(hsv, hsv, blurSize);

        Core.inRange(hsv, new Scalar(0, 0, 0), new Scalar(255, 255, 30), blackMask);

        Imgproc.findContours(blackMask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

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

            Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));

            Imgproc.rectangle(mat, boundingRect, new Scalar(40, 200, 0), 10);

            xPosition = boundingRect.x + (boundingRect.width * 0.5);
            yPosition = boundingRect.y + (boundingRect.height * 0.5);
        }

        contours = new ArrayList<>();
        hierarchy = new Mat();

        return mat;
    }
}
