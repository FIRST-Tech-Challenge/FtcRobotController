package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Size;

public class GrabberCameraPipeline extends OpenCvPipeline {
    double[] black = {200, 200, 200};
    double[] white = {235, 235, 235};

    public int junctionX = 0;
    public int junctionY = 0;
    public double distanceToCenterX = 0;
    public double distanceToCenterY = 0;
    public double contourSize = 0.0;
    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    private Mat hsv = new Mat();
    private Mat blackMask = new Mat();
    private Size blurSize = new Size(5,5);

    @Override
    public Mat processFrame(Mat mat) {
        Imgproc.cvtColor(mat,hsv,Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(hsv,hsv, blurSize);
        Core.inRange(hsv, new Scalar(0, 0, 0), new Scalar(255, 255, 30), blackMask);
        Imgproc.findContours(blackMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 0) {
            double maxVal = 0.0;
            int maxValIdx = -1;

            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));

                if (maxVal < contourArea && contourArea < 18000) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
            if(maxValIdx != -1){
            // draw bounding rectangle around the largest contour
            Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));
            Imgproc.rectangle(mat, boundingRect, new Scalar(40, 200, 0), 10);
            junctionX = boundingRect.x + (boundingRect.width / 2);
            junctionY = boundingRect.y + (boundingRect.height / 2);
            contourSize = maxVal;
            distanceToCenterX = junctionX - Constants.CAMERA_CENTER_X;
            distanceToCenterY = junctionY - Constants.CAMERA_CENTER_Y;
        }else{}}
        contours = new ArrayList<>();
        hierarchy = new Mat();
        return mat;
    }
}
