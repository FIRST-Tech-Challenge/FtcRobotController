package org.firstinspires.ftc.teamcode.toolkit.opencvtoolkit;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingDetector extends OpenCvPipeline {

    public int ringCount = -1;
    public double rectWidth;
    public double rectHeight;
    public double rectRatio;
    public double totalMatArea;

    @Override
    public Mat processFrame(Mat input) {

        // zoom in on the input image
        input = zoomMat(input, 1.5, new Point(input.width() * 0.5, input.height() * 0.667));
        totalMatArea = input.width() * input.height();
        Mat zoomedMat = input.clone();

        // blur zoomed image and convert it to HSV
        Imgproc.blur(zoomedMat, zoomedMat, new Size(2.0, 2.0), new Point(-1, -1));
        Imgproc.cvtColor(zoomedMat, zoomedMat, Imgproc.COLOR_RGB2HSV);

        // set the HSV limits and filter out the other colors (everything but orange)
        Scalar lowHSV = new Scalar(10, 100, 80);
        Scalar highHSV = new Scalar(20, 255, 255);
        Core.inRange(zoomedMat, lowHSV, highHSV, zoomedMat);

        // convert Mat to binary
        Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
        Imgproc.threshold(zoomedMat, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        //Finding Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        List<MatOfPoint> prunedContours = pruneContours(contours);

        // draw the normal contours in blue
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 0, 255), 2, Imgproc.LINE_8);

        // draw the pruned (most important) contours in green
        Imgproc.drawContours(input, prunedContours, -1, new Scalar(0, 255, 0), 2, Imgproc.LINE_8);

        // determine the largest contour in the shortened contour list
        if(!prunedContours.isEmpty()) {

            double maxArea = 0;
            int maxIndex = 0;

            for (int i = 0; i < prunedContours.size(); i++) {
                double area = Imgproc.contourArea(prunedContours.get(i));
                if (area > maxArea) {
                    maxArea = Imgproc.contourArea(prunedContours.get(i));
                    maxIndex = i;
                }
            }

            MatOfPoint largestContour = prunedContours.get(maxIndex);

            // Transform the contour to a different format
            Point[] points = largestContour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(largestContour.toArray());

            // Do a rect fit to the contour, and draw it on the screen (both zoomedMat and input)
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRectOnObject(rotatedRectFitToContour, input);
            drawRectOnObject(rotatedRectFitToContour, zoomedMat);

            rectWidth = rotatedRectFitToContour.size.width;
            rectHeight = rotatedRectFitToContour.size.height;

            if (rectWidth < rectHeight) {
                double temp = rectHeight;
                rectHeight = rectWidth;
                rectWidth = temp;
            }

            rectRatio = rectHeight / rectWidth;

            // Determine the number of rings detected
            // if rectangle height : width ratio is between 2.5/5 and 4/5, then there are 4 rings (5 x 3 inches)
            if (rectRatio > 0.5 && rectRatio < 0.7) {
                ringCount = 4;
            }
            // if height : width ratio is between 0.5/5 and 2/5, then there is 1 ring (5 x 0.75 inches)
            else if (rectRatio > 0.15 && rectRatio < 0.35) {
                ringCount = 1;
            } else {
                ringCount = -1;
            }

        } else {
            ringCount = 0;
        }

        return input;

    }

    // method to draw the rectangle around the chosen object
    static void drawRectOnObject(RotatedRect rect, Mat drawOn)
    {
        Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], new Scalar(255, 0, 0), 2);
        }
    }

    // method to crop the mat (image) centered around a given point
    public Mat zoomMat(Mat inputMat, double zoomAmount, Point pt) {
        double width = inputMat.width();
        double height = inputMat.height();

        double zoomWidth = width / zoomAmount;
        double zoomHeight = height / zoomAmount;

        Range columnRange = new Range((int)(pt.x - (0.5 * zoomWidth)), (int)(pt.x + (0.5 * zoomWidth)));
        Range rowRange = new Range((int)(pt.y - (0.5 * zoomHeight)), (int)(pt.y + (0.5 * zoomHeight)));

        return new Mat(inputMat, rowRange, columnRange);
    }

    // method to crop the mat (image) so that the center is only seen (takes a number like 1, 2, 3 as the zoom amount)
    public Mat zoomMat(Mat inputMat, double zoomAmount) {
        double width = inputMat.width();
        double height = inputMat.height();

        double zoomWidth = width / zoomAmount;
        double zoomHeight = height / zoomAmount;

        Range columnRange = new Range((int)(0.5 * (width - zoomWidth)), (int)(width - (0.5 * (width - zoomWidth))));
        Range rowRange = new Range((int)(0.5 * (height - zoomHeight)), (int)(height - (0.5 * (height - zoomHeight))));

        return new Mat(inputMat, rowRange, columnRange);
    }

    // method to shorten list of contours to contours that are not too big or too small to be the rings
    public List<MatOfPoint> pruneContours(List<MatOfPoint> contours) {
        List<MatOfPoint> prunedContours = new ArrayList<>();

        for(int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if(area < (0.75 * totalMatArea) && (area > (0.01 * totalMatArea) || area > 100)) {
                prunedContours.add(contours.get(i));
            }
        }

        return prunedContours;
    }

}

