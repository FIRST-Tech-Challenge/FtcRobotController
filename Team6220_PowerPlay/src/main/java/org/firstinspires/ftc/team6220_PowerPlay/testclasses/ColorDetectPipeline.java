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
import java.util.List;

public class ColorDetectPipeline extends OpenCvPipeline {
    Size blurSize = new Size(49, 49);
    double erodeSize = 220;
    boolean isRunning = false;
    Rect detectedRect = new Rect();
    int counter = 0;
    boolean rectDetected = false;

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

        if (input == null) {
            throw new IllegalArgumentException("Input cannot be null");
        }

        isRunning = true;
        int[] ca = {43, 200, 127};
        int[] co = {100, 50, 127};
        counter++;
        return processFrameWithRange(input, ca, co);
    }

    public Mat processFrameWithRange(Mat input, int[] colorTarget, int[] colorTolerance) throws IllegalArgumentException {
        isRunning = true;
        if (input == null) {
            throw new IllegalArgumentException("Input cannot be null");
        }

        Mat frame = input;

        // Convert color to HSV
        Imgproc.cvtColor(input, frame, Imgproc.COLOR_BGR2HSV);

        // Threshold the image
        frame = targetToleranceMatte(frame, colorTarget, colorTolerance);

        // Blur and then threshold to remove small details and sort of "erode" the matte
        Mat output1 = null;
        Imgproc.GaussianBlur(frame, output1, blurSize, 0);
        Mat output2 = null;
        Imgproc.threshold(frame, output2, erodeSize, 255, Imgproc.THRESH_BINARY);

        // Get the contours of the image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find largest contour and then draw a rectangle around it for display
        double maxArea;
        int maxAreaContour;
        if (contours.size() > 0) {
            maxArea = 0.0;
            maxAreaContour = -1;

            for (int i = 0; i < contours.size(); i++) {
                if (Imgproc.contourArea(contours.get(i)) > maxArea) {
                    maxArea = Imgproc.contourArea(contours.get(i));
                    maxAreaContour = i;
                }
            }

            detectedRect = Imgproc.boundingRect(contours.get(maxAreaContour));
            Imgproc.rectangle(input, detectedRect, new Scalar(40, 200, 0), 10);

            // Transform the detected rectangle's coordinates so that (0, 0) is at the center of the image,
            // and instead of detectedRect.x and detectedRect.y corresponding to the top-left corner they correspond to the center of the rectangle
            detectedRect.x -= input.width() * 0.5 - detectedRect.width * 0.5;
            detectedRect.y -= input.height() * 0.5 - detectedRect.height * 0.5;
        }

        rectDetected = contours.size() > 0;
        counter = contours.size();
        Mat finalOutput = null;
        Imgproc.cvtColor(input, finalOutput, Imgproc.COLOR_HSV2BGR);

        frame = null;
        contours = null;
        hierarchy = null;

        return finalOutput;
    }
}
