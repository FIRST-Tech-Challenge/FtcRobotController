package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ColorDetectPipeline extends OpenCvPipeline {
    private Size blurSize = new Size(25, 25);
    private double erodeSize = 220;
    private int[] ca;
    private int[] co;

    public boolean isRunning = false;
    public Rect detectedRect = new Rect();
    public boolean rectDetected = false;
    public int counter = 0;
    public double biggestContourArea = 0;
    public double biggestContourBoundingBoxArea = 0;

    private Mat frame = new Mat();
    private Mat matte = new Mat();
    private Mat blurMatte = new Mat();
    private Mat thresholdMatte = new Mat();
    private Mat hierarchy = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat finalOutput = new Mat();

    public ColorDetectPipeline(int[] colorTarget, int[] colorTolerance) {
        ca = colorTarget;
        co = colorTolerance;
    }

    double RectArea(Rect r) {
        return r.width*r.height;
    }

    Mat targetToleranceMatte(Mat img, int[] ca, int[] co) {
        double[] lower = new double[3], upper = new double[3];

        for (int i = 0; i < ca.length; i++) {
            lower[i] = ca[i] - co[i];
            upper[i] = ca[i] + co[i];
        }

        Mat dst = new Mat();
        Core.inRange(img, new Scalar(lower), new Scalar(upper), dst);
        return dst;
    }

    @Override
    public Mat processFrame(Mat input) throws IllegalArgumentException {
        return processFrameWithRange(input, ca, co);
    }

    public Mat processFrameWithRange(Mat input, int[] colorTarget, int[] colorTolerance) throws IllegalArgumentException {
        isRunning = true;
        if (input == null) {
            throw new IllegalArgumentException("Input cannot be null");
        }
        counter++;

        // Convert color to HSV
        Imgproc.cvtColor(input, frame, Imgproc.COLOR_RGB2HSV);

        // Threshold the image
        matte = targetToleranceMatte(frame, colorTarget, colorTolerance);

        // Blur and then threshold to remove small details and sort of "erode" the matte
        Imgproc.GaussianBlur(matte, blurMatte, blurSize, 0);
        //Imgproc.cvtColor(matte, matte, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(blurMatte, thresholdMatte, erodeSize, 255, Imgproc.THRESH_BINARY);


        // Get the contours of the image
        Imgproc.findContours(thresholdMatte, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0) {
            double maxArea = 0.0;
            int maxAreaContour = 0;

            // Find the biggest contour
            for (int i = 0; i < contours.size(); i++) {
                if (Imgproc.contourArea(contours.get(i)) > maxArea) {
                    maxArea = Imgproc.contourArea(contours.get(i));
                    maxAreaContour = i;
                }
            }

            // Find the bounding box of the biggest contour
            detectedRect = Imgproc.boundingRect(contours.get(maxAreaContour));

            // Calculate the area of both the biggest contour and the bounding box
            biggestContourBoundingBoxArea = RectArea(detectedRect);
            biggestContourArea = Imgproc.contourArea(contours.get(maxAreaContour));

            // Draw the bounding box
            Imgproc.rectangle(input, detectedRect, new Scalar(40, 200, 0), 10);

            // Transform the detected rectangle's coordinates so that (0, 0) is at the center of the image,
            // and instead of detectedRect.x and detectedRect.y corresponding to the top-left corner they correspond to the center of the rectangle
            detectedRect.x -= (input.width() - detectedRect.width) * 0.5;
            detectedRect.y -= (input.height() - detectedRect.height) * 0.5;
        }

        rectDetected = contours.size() > 0;
        counter = contours.size();

        contours = new ArrayList<>();
        hierarchy = new Mat();
        return input;
    }
}
