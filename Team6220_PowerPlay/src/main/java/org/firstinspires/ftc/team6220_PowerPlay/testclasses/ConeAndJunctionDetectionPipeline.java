package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ConeAndJunctionDetectionPipeline extends OpenCvPipeline {
    // range of colors for blue, red, and yellow objects
    private final Scalar lowerBlue = new Scalar(0, 130, 100);
    private final Scalar upperBlue = new Scalar(90, 190, 130);
    private final Scalar lowerRed = new Scalar(30, 100, 170);
    private final Scalar upperRed = new Scalar(130, 140, 220);
    private final Scalar lowerYellow = new Scalar(90, 80, 160);
    private final Scalar upperYellow = new Scalar(160, 120, 190);

    private final Scalar blue = new Scalar(0, 0, 255);
    private final Scalar red = new Scalar(255, 0, 0);
    private final Scalar yellow = new Scalar(255, 255, 0);

    Mat mat = new Mat();

    Mat blueMask = new Mat();
    Mat redMask = new Mat();
    Mat yellowMask = new Mat();

    ArrayList<MatOfPoint> blueContours = new ArrayList<>();
    ArrayList<MatOfPoint> redContours = new ArrayList<>();
    ArrayList<MatOfPoint> yellowContours = new ArrayList<>();

    Mat blueHierarchy = new Mat();
    Mat redHierarchy = new Mat();
    Mat yellowHierarchy = new Mat();

    int largestBlueContour;
    int largestRedContour;
    int largestYellowContour;

    double blueContourArea;
    double redContourArea;
    double yellowContourArea;

    double largestBlueContourArea;
    double largestRedContourArea;
    double largestYellowContourArea;

    Rect blueRectangle = new Rect();
    Rect redRectangle = new Rect();
    Rect yellowRectangle = new Rect();

    @Override
    public Mat processFrame(Mat input) {
        // convert the frame to the YCrCb color space
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2YUV);

        // apply median blur on the frame
        Imgproc.medianBlur(mat, mat, 9);

        // separately mask the frame for each color
        Core.inRange(mat, lowerBlue, upperBlue, blueMask);
        Core.inRange(mat, lowerRed, upperRed, redMask);
        Core.inRange(mat, lowerYellow, upperYellow, yellowMask);

        // find all of the contours for each color
        Imgproc.findContours(blueMask, blueContours, blueHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(redMask, redContours, redHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(yellowMask, yellowContours, yellowHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // find the largest contour for blue
        largestBlueContourArea = 0.0;

        for (int i = 0; i < blueContours.size(); i++) {
            blueContourArea = Imgproc.contourArea(blueContours.get(i));

            if (blueContourArea > largestBlueContourArea) {
                largestBlueContourArea = blueContourArea;
                largestBlueContour = i;
            }
        }

        // find the largest contour for red
        largestRedContourArea = 0.0;

        for (int i = 0; i < redContours.size(); i++) {
            redContourArea = Imgproc.contourArea(redContours.get(i));

            if (redContourArea > largestRedContourArea) {
                largestRedContourArea = redContourArea;
                largestRedContour = i;
            }
        }

        // find the largest contour for yellow
        largestYellowContourArea = 0.0;

        for (int i = 0; i < yellowContours.size(); i++) {
            yellowContourArea = Imgproc.contourArea(yellowContours.get(i));

            if (yellowContourArea > largestYellowContourArea) {
                largestYellowContourArea = yellowContourArea;
                largestYellowContour = i;
            }
        }

        // find and draw the bounding box for the blue color
        if (blueContours.size() > 0) {
            blueRectangle = Imgproc.boundingRect(blueContours.get(largestBlueContour));
            Imgproc.rectangle(input, blueRectangle, blue, 5);
        }

        // find and draw the bounding box for the red color
        if (redContours.size() > 0) {
            redRectangle = Imgproc.boundingRect(redContours.get(largestRedContour));
            Imgproc.rectangle(input, redRectangle, red, 5);
        }

        // find and draw the bounding box for the yellow color
        if (yellowContours.size() > 0) {
            yellowRectangle = Imgproc.boundingRect(yellowContours.get(largestYellowContour));
            Imgproc.rectangle(input, yellowRectangle, yellow, 5);
        }

        return input;
    }
}
