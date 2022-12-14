package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorDetectPipeline extends OpenCvPipeline {
    public boolean isRunning = false;
    Size blurSize = new Size(49, 49);
    double erodeSize = 220;

    public Rect detectedRect = new Rect();
    public int counter = 0;

    Mat targetToleranceMatte(Mat img, int[] ca, int[] co) {
        double[] lower = new double[3], upper = new double[3];
        for(int i = 0; i < ca.length; i++) {
            lower[i] = ca[i]-co[i];
            upper[i] = ca[i]+co[i];
        }
        Mat dst = new Mat();
        Core.inRange(img, new Scalar(lower), new Scalar(upper), dst);
        return dst;
    }

    MatOfPoint getBiggestContour(List<MatOfPoint> contours) {
        if(contours.isEmpty()) {
            return new MatOfPoint();
        }
        if(contours.size() == 1) {
            return contours.get(0);
        }
        double maxArea = -1;
        MatOfPoint maxAreaContour = new MatOfPoint();

        for(MatOfPoint c : contours) {
            double currentArea = Imgproc.contourArea(c);
            if(currentArea > maxArea){
                maxArea = currentArea;
                maxAreaContour = c;
            }
        }

        return maxAreaContour;
    }

    Rect getBiggestContourBoundingBox(List<MatOfPoint> contours) {
        if(contours.isEmpty()) {
            return new Rect(new Point(0, 0), new Point(0,0));
        }
        double maxArea = -1;
        MatOfPoint maxAreaContour = new MatOfPoint();

        for(MatOfPoint c : contours) {
            double currentArea = Imgproc.contourArea(c);
            if(currentArea > maxArea){
                maxArea = currentArea;
                maxAreaContour = c;
            }
        }

        return Imgproc.boundingRect(maxAreaContour);
    }

    @Override
    public Mat processFrame(Mat input)/* throws IllegalArgumentException */{

        if(input == null) { throw new IllegalArgumentException("Input cannot be null"); }
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
        Imgproc.GaussianBlur(frame, frame, blurSize, 0);
        Imgproc.threshold(frame, frame, erodeSize, 255, Imgproc.THRESH_BINARY);

        // Get the contours of the image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

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
            detectedRect.x -= input.width()*0.5-detectedRect.width*0.5;
            detectedRect.y -= input.height()*0.5-detectedRect.height*0.5;
        }
        counter = contours.size();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2BGR);

        return input;
    }
}
