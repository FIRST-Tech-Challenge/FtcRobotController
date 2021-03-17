package org.firstinspires.ftc.team417_2020;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingDetectionOpenCV extends OpenCvPipeline {

    private Mat displayMat = new Mat();
    private Mat yuv = new Mat();
    private Mat u = new Mat();
    private Mat blurred = new Mat();

    private Mat hierarchy = new Mat();
    Rect rectCrop = new Rect(230, 300, 180, 130);

    Rect rect = new Rect();
    Rect maxRect = new Rect(0,0,1,1);

    int uThreshold = 110;

    @Override
    public Mat processFrame(Mat input) {

        // copy input frame to different Mat as to not alter input
        input.copyTo(displayMat);
        // blur image
        Imgproc.GaussianBlur(input, blurred, new Size(15,15), 0);
        // convert color space to YUV
        Imgproc.cvtColor(blurred, yuv,Imgproc.COLOR_BGR2YUV);
        // split blurred YUV image into YUV channels

        Core.extractChannel(yuv,u,2);
        //u = u.submat(rectCrop);
        Imgproc.threshold(u, u, uThreshold,255,Imgproc.THRESH_BINARY_INV);
        List<MatOfPoint> contours = new ArrayList<>();
        // find contours
        Imgproc.findContours(u, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // find largest contour and assign to maxRect
        double maxArea = 0.0;
        for (MatOfPoint contour: contours){
            rect = Imgproc.boundingRect(contour);
            Imgproc.rectangle(displayMat,
                    new Point(rect.x, rect.y),
                    new Point(rect.x + rect.width, rect.y + rect.height),
                    new Scalar(0,0,0), 4);
            double area = Imgproc.contourArea(contour);
            if (area > maxArea){
                maxArea = area;
                maxRect = rect;
            }
        }
        double x = maxRect.x;
        double y = maxRect.y;
        // draw rectangle around largest contour
        Imgproc.rectangle(displayMat,
                new Point(x, y),
                new Point(x + maxRect.width, y + maxRect.height),
                new Scalar(0,0,0), 4);
        Imgproc.putText(displayMat, "" + maxRect.toString(), new Point(x,y), 0, 1, new Scalar(0,0,0));

        return displayMat;
    }
}
