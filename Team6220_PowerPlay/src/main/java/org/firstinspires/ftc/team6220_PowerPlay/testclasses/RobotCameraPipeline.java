package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

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
    //Baller fields
    public double Xpos = 0.0;
    public double Ypos = 0.0;
    public double width = 0.0;
    public double longAxis = 0.0;
    public double shortAxis = 0.0;
    public double distance = 0;
    public double coneSize = 0;
    public double coneSizeContourArea = 0;
    public boolean grab = false;
    private int[] lowerRange = {42, 128, 114};
    private int[] upperRange = {168, 242, 255};

    //Turns int[] into scalar();
    Scalar intToScalar(int[] a) {
        if (a.length != 3) {
            throw new IllegalArgumentException("Input must be a three-item array");
        }

        return new Scalar(a[0], a[1], a[2]);
    }

    //Masks the a frame based on low and high HSV ranges
    public Mat maskFrame(Mat input, int[] lower, int[] upper) {
        //Mat used for the HSV colorspace transtion output
        Mat HSV = new Mat();
        //Set blur size
        Size blurSize = new Size(5, 5);
        //Transition to HSV colorspace
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);
        //Blur the HSV frame based on the blur size
        Imgproc.blur(HSV, HSV, blurSize);
        //Mask the frame based on the ranges
        Core.inRange(HSV, intToScalar(lower), intToScalar(upper), HSV);
        return HSV;
    }

    //Method used to set the ranges for the pipeline, it uses fields because of the way the class structure is set up.
    public void setRanges(int[] lower, int[] upper) {
        lowerRange = lower;
        upperRange = upper;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (input == null) {
            throw new IllegalArgumentException("Input cannot be null");
        }
        //Mask the actual input frame based on the upperRange and lowerRange fields, which are changes by setRanges()
        Mat mask = maskFrame(input, lowerRange, upperRange);

        //Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //Find the largest contour if there is one
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
            //Draw bounding rectangle around the largest contour
            Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));
            //Get moo - Reeeeeeeeeee(se)
            Moments m = Imgproc.moments(contours.get(maxValIdx), false);
            //Draw the bounding rectangle on the camera stream
            Imgproc.rectangle(input, boundingRect, new Scalar(40, 200, 0), 10);
            //Save coordinates of the bounding box to fields, if there is a bounding box
            if (m.get_m00() > 0) {
                double cX = boundingRect.x + (boundingRect.width / 2);
                distance = cX - Constants.CAMERA_CENTER_X;
                Xpos = boundingRect.x + (boundingRect.width / 2);
                Ypos = boundingRect.y + (boundingRect.height / 2);
                width = boundingRect.width;
                if(boundingRect.height > boundingRect.width){
                    shortAxis = boundingRect.width;
                    longAxis = boundingRect.height;
                }else{
                    shortAxis = boundingRect.height;
                    longAxis = boundingRect.width;
                }
                coneSize = boundingRect.width * boundingRect.height;
                coneSizeContourArea = maxVal;
                grab = Math.abs(distance) < 30 && coneSize < 3000;
            }
        }

        return input;
    }
}
