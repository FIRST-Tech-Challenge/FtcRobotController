package org.firstinspires.ftc.team6220_CENTERSTAGE;

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

public class ColorDetectionPipeline extends OpenCvPipeline
{
    boolean viewportPaused = false;
    Mat input = new Mat();
    Mat threshold = new Mat();
    Mat contourMask = new Mat();
    Mat output = new Mat();
    Scalar minRange = new Scalar(0, 0, 0);
    Scalar maxRange = new Scalar(255, 255, 255);

    double centerPosX = 0;
    double centerPosY = 0;

    public enum PropPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public void setRanges(Scalar minRange, Scalar maxRange) {
        this.minRange = minRange;
        this.maxRange = maxRange;
    }

    public PropPosition returnZone() {
        if (centerPosX <= (Constants.CAMERA_WIDTH)/3) {
            return PropPosition.LEFT;
        } else if (centerPosX <= (Constants.CAMERA_WIDTH/3)*2) {
            return PropPosition.MIDDLE;
        } else {
            return PropPosition.RIGHT;
        }
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // convert image to grayscale
        Imgproc.cvtColor(input, this.input, Imgproc.COLOR_RGB2HSV);
        // blur the image to reduce the impact of noisy pixels
        Imgproc.GaussianBlur(this.input, this.input, new Size(7,7),0);
        Core.inRange(this.input, this.minRange, this.maxRange, this.input);
        Imgproc.threshold(this.input, threshold, 1, 255,Imgproc.THRESH_BINARY);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(threshold, contours, contourMask, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() > 1) {
            double maxArea = 0;
            int maxAreaContourIndex = -1;
            for (int i = 0; i < contours.size(); i++) {
                if (Imgproc.contourArea(contours.get(i)) > maxArea) {
                    maxArea = Imgproc.contourArea(contours.get(i));
                    maxAreaContourIndex = i;
                }
            }

            Imgproc.drawContours(input, contours, maxAreaContourIndex, Constants.borderColors, 2, -1);

            Rect boundingRect = Imgproc.boundingRect(contours.get(maxAreaContourIndex));
            double boundHeightX = boundingRect.x + boundingRect.width;
            double boundHeightY = boundingRect.y + boundingRect.height;
            centerPosX = (int)boundingRect.width/2 + boundingRect.x;
            centerPosY = (int)boundingRect.height/2 + boundingRect.y;
            Point circlePoint = new Point((centerPosX), (centerPosY));
            Imgproc.rectangle(input, new Point(boundingRect.x, boundingRect.y), new Point(boundHeightX, boundHeightY), Constants.borderColors, 1, Imgproc.LINE_8, 0);
            Imgproc.circle(input, circlePoint , 10, Constants.borderColors , Imgproc.LINE_8, -1);
        }

        return input;
    }

}