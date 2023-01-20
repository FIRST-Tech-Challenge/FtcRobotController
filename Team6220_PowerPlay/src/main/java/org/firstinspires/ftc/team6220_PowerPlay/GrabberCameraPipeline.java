package org.firstinspires.ftc.team6220_PowerPlay;

import org.opencv.core.Core;
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
    Mat circleThresh = new Mat();
    Mat mask = new Mat();
    Point centerPoint = new Point(Constants.CAMERA_CENTER_X,Constants.CAMERA_CENTER_Y);

    @Override
    public Mat processFrame(Mat input) {
        //make the circle that will be used to crop the image
        Imgproc.circle(circleThresh, centerPoint, 200, new Scalar(255,255,255), -1, 8, 0 );

        //crop the input image based on the circle
        input.copyTo(mask,circleThresh);

        // transform the RGB frame into a HSV frame
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        // blur the HSV frame
        Imgproc.GaussianBlur(mask, mask, Constants.BLUR_SIZE, 0);

        // mask the blurred frame
        Core.inRange(mask, Constants.LOWER_BLACK, Constants.UPPER_BLACK, mask);

        // find the contours in the masked frame
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // find the largest contour if there is one
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

            // get the bounding rectangle around the largest contour
            Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));

            // get moments
            Moments moments = Imgproc.moments(contours.get(maxValIdx), false);

            // draw the bounding rectangle on the frame
            Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 10);

            if (moments.get_m00() > 0) {
                xPosition = boundingRect.x + (boundingRect.width * 0.5);
                yPosition = boundingRect.y + (boundingRect.height * 0.5);
            }
        } else {
            xPosition = Constants.CAMERA_CENTER_X;
            yPosition = Constants.CAMERA_CENTER_Y;
            detected = false;
        }

        contours.clear();
        return mask;
    }
}
