package org.firstinspires.ftc.team6220_PowerPlay;

import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RobotCameraPipeline extends OpenCvPipeline {
    public double xPosition = 0.0;
    public double yPosition = 0.0;
    public double width = 0.0;

    private int[] lowerRange = {42, 128, 114};
    private int[] upperRange = {168, 242, 255};

    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    Mat HSV = new Mat();
    Mat mask = new Mat();
    Size blurSize = new Size(5, 5);

    // turns an int array into a scalar
    Scalar intToScalar(int[] a) {
        if (a.length != 3) {
            throw new IllegalArgumentException();
        }

        return new Scalar(a[0], a[1], a[2]);
    }

    // masks the frame based on low and high HSV ranges
    public Mat maskFrame(Mat input, int[] lower, int[] upper) {
        // transition to HSV color space
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);

        // blur the HSV frame based on the blur size
        Imgproc.GaussianBlur(HSV, HSV, blurSize,0);

        // mask the frame based on the ranges
        Core.inRange(HSV, intToScalar(lower), intToScalar(upper), HSV);

        return HSV;
    }

    // method used to set the ranges for the pipeline
    public void setRanges(int[] lower, int[] upper) {
        lowerRange = lower;
        upperRange = upper;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (input == null) {
            throw new IllegalArgumentException();
        }

        // mask the actual input frame based on the ranges, which can be changed by setRanges()
        mask = maskFrame(input, lowerRange, upperRange);

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // find the largest contour if there is one
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

            // draws bounding rectangle around the largest contour
            Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));

            // get moments
            Moments moments = Imgproc.moments(contours.get(maxValIdx), false);

            // draws the bounding rectangle on the frame
            Imgproc.rectangle(input, boundingRect, new Scalar(40, 200, 0), 10);

            // saves coordinates of the bounding box to fields if there is a bounding box
            if (moments.get_m00() > 0) {
                xPosition = boundingRect.x + (boundingRect.width * 0.5);
                yPosition = boundingRect.y + (boundingRect.height * 0.5);
                width = boundingRect.width;
            }
        }

        contours.clear();
        return input;
    }
}
