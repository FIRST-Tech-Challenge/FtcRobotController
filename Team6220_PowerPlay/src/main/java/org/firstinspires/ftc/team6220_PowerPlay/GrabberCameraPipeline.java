package org.firstinspires.ftc.team6220_PowerPlay;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class GrabberCameraPipeline extends OpenCvPipeline {
    public double xPosition = Constants.CAMERA_CENTER_X;
    public double yPosition = Constants.CAMERA_CENTER_Y;
    public boolean detected = false;

    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    Mat mat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Imgproc.GaussianBlur(mat, mat, Constants.BLUR_SIZE, 0);

        Core.inRange(mat, Constants.LOWER_BLACK, Constants.UPPER_BLACK, mat);

        Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            detected = true;
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

            xPosition = boundingRect.x + (boundingRect.width * 0.5);
            yPosition = boundingRect.y + (boundingRect.height * 0.5);

            double distanceFromCenter = Math.sqrt(Math.pow(Math.abs((xPosition - Constants.CAMERA_CENTER_X)), 2) + Math.pow(Math.abs((yPosition - Constants.CAMERA_CENTER_Y)), 2));

            if (distanceFromCenter < 250) {
                Moments moments = Imgproc.moments(contours.get(maxValIdx), false);

                Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 10);

                if (moments.get_m00() > 0) {
                    xPosition = boundingRect.x + (boundingRect.width * 0.5);
                    yPosition = boundingRect.y + (boundingRect.height * 0.5);
                }
            }
        } else {
            detected = false;
            xPosition = Constants.CAMERA_CENTER_X;
            yPosition = Constants.CAMERA_CENTER_Y;
        }

        contours.clear();
        return input;
    }
}
