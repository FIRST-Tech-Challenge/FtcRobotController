package org.firstinspires.ftc.teamcode.Components.CV;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class ConeStackObserverPipeline extends OpenCvPipeline {
    ArrayList<Double> frameList;
    double coneStackCenter = 0;

    public ConeStackObserverPipeline() {
        frameList=new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        Scalar lowHSV = new Scalar(111.3, 70, 60);
        Scalar highHSV = new Scalar(114.5, 255, 255);

        Mat thresh = new Mat();

        // Get a black and white image of blue objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours of edges
        Imgproc.findContours(masked, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        //rotatedRect because it allows for more accurate bounding rectangles, perfect if pole is slanted
        RotatedRect[] rectangle = new RotatedRect[contours.size()];
        //iterate through each contour
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            //convert contour to approximate polygon
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 10, true);
            //find rotatedRect for polygon
            rectangle[i] = Imgproc.minAreaRect(contoursPoly[i]);
        }
        //find index of largest rotatedRect(assumed that it is closest tile)
        int maxAreaIndex = 0;
        double maxWidth = 0;
        //iterate through each rotatedRect find largest
        for (int i = 1; i < rectangle.length; i++) {
            if(rectangle[i].size.height<rectangle[i].size.width){
                if (rectangle[i].size.height > maxWidth) {
                    maxAreaIndex = i;
                    maxWidth = rectangle[i].size.height;
                }
            }
            else{
                if (rectangle[i].size.width > maxWidth) {
                    maxAreaIndex = i;
                    maxWidth = rectangle[i].size.width;
                }
            }
        }
        //if there is a detected largest contour, record information about it
        if(rectangle.length>0) {
            if(rectangle[maxAreaIndex].size.height<rectangle[maxAreaIndex].size.width) {
                coneStackCenter = rectangle[maxAreaIndex].center.y - 320;
            }
            else{
                coneStackCenter = rectangle[maxAreaIndex].center.x - 320;

            }
            frameList.add(coneStackCenter);
        }
        //list of frames to reduce inconsistency, not too many so that it is still real-time
        if(frameList.size()>5) {
            frameList.remove(0);
        }
        input.release();
        mat.release();
        masked.release();
        thresh.release();
        hierarchy.release();
        Scalar lineColor= new Scalar(255,50,50);
        if(contoursPoly.length>0) {
            Imgproc.rectangle(input, Imgproc.boundingRect(contoursPoly[maxAreaIndex]), lineColor, 5);
        }
        return input;
    }
    public double centerOfPole() {
        double average=0;
        for(int i=0;i<frameList.size();i++){
            average+=frameList.get(i);
        }
        return average/frameList.size();
    }
    public double getAngleOfCone(){
        return Math.atan(((centerOfPole()-320)/715) * (180/Math.PI));
    }
}