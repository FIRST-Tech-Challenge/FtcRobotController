package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class SkystoneDetectionPipeline extends OpenCvPipeline
{
    public Point targetLocation = new Point();
    public double contourArea = 0;
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */


        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */
        //TODO: change the threshold values for your purposes
        Mat mask = threshold(input, new Scalar(0, 130, 200), new Scalar(179, 160, 255));
        Mat kernel = new Mat(new Size(7, 7), CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        List<MatOfPoint> contours = getContours(mask);
        int lineNum = 0;
        try {
            MatOfPoint selected = contours.get(0);
            for (MatOfPoint contour : contours){
                if (Imgproc.contourArea(contour) > Imgproc.contourArea(selected)){
                    selected = contour;
                }
            }
            contourArea = Imgproc.contourArea(selected);
            List<MatOfPoint> listSelected  = new ArrayList<>();
            listSelected.add(selected);
            Imgproc.drawContours(input, listSelected, 0, new Scalar(255, 0, 0), 2);
            MatOfPoint2f selected2f = new MatOfPoint2f(selected.toArray());
            RotatedRect boundRect = Imgproc.minAreaRect(selected2f);
            Imgproc.rectangle(input, boundRect.boundingRect(), new Scalar(0, 255, 0));
            targetLocation = boundRect.center;
            Imgproc.circle(input, targetLocation, 2, new Scalar(0, 0, 255), 2);
        }catch (Exception e){
            Log.e("OpenCVDebuggingStuff", e.getMessage() + " " + lineNum);
        }
        return mask;
    }
    public Mat threshold(Mat input, Scalar low, Scalar high){
        Mat newMat = new Mat(input.size(), CvType.CV_8U);
        Mat hsv = new Mat(input.size(), CvType.CV_32F);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, low, high, newMat);
        return newMat;
    }
    public List<MatOfPoint> getContours(Mat mask){
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.CHAIN_APPROX_SIMPLE, Imgproc.RETR_TREE);
        return contours;
    }

    public Point getTargetLocation(){
        return targetLocation;
    }

    public double getContourArea(){
        return contourArea;
    }
}