package org.firstinspires.ftc.teamcode.Components.CV;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
@Config
public class ConeObserverPipeline extends OpenCvPipeline {
    public static double  degPerPix = 22.5/320, widTimesDist = 820*4*24/31.0*12/8.7*13.5/14.9, focalLength = 715;
    double centerOfPole = 0, poleSize = 0;
    ArrayList<double[]> frameList;
    public static double LowS = 90;
    public static double HighS = 255;
    public static double LowH = 110;
    public static double HighH = 125;
    public static double LowV = 0;
    public static double HighV = 255;
    public static double LowS1 = 50;
    public static double HighS1 = 255;
    public static double LowH1 = 210;
    public static double HighH1 = 255;
    public static double LowV1 = 0;
    public static double HighV1 = 255;
    public static double LowS2 = 50;
    public static double HighS2 = 255;
    public static double LowH2 = 0;
    public static double HighH2 = 15;
    public static double LowV2 = 0;
    public static double HighV2 = 255;


    public ConeObserverPipeline() {
        frameList=new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        Scalar lowHSV = new Scalar(LowH, LowS, LowV); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(HighH, HighS, HighV); // lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

//        Scalar lowHSV1 = new Scalar(LowH1, LowS1, LowV1); // lenient lower bound HSV for yellow
//        Scalar highHSV1 = new Scalar(HighH1, HighS1, HighV1); // lenient higher bound HSV for yellow
//
//        Mat thresh2 = new Mat();
//
//        // Get a black and white image of yellow objects
//        Core.inRange(mat, lowHSV1, highHSV1, thresh2);
//        Scalar lowHSV2 = new Scalar(LowH2, LowS2, LowV2); // lenient lower bound HSV for yellow
//        Scalar highHSV2 = new Scalar(HighH2, HighS2, HighV2); // lenient higher bound HSV for yellow
//
//        Mat thresh3 = new Mat();
//
//        // Get a black and white image of yellow objects
//        Core.inRange(mat, lowHSV2, highHSV2, thresh3);
//
//        Mat thresh4 = new Mat();
//
//        Core.bitwise_and(thresh, thresh2, thresh4);
//        Core.add(thresh,thresh2,thresh4);
//
//        Mat thresh5 = new Mat();
//
//        Core.add(thresh4, thresh3, thresh5);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours of edges
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        //rotatedRect because it allows for more accurate bounding rectangles, perfect if pole is slanted
        Rect[] rectangle = new Rect[contours.size()];

        //iterate through each contour
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            //convert contour to approximate polygon
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 10, true);
            //find rotatedRect for polygon
            rectangle[i] = Imgproc.boundingRect(contoursPoly[i]);
        }
//
//        //find index of largest rotatedRect(assumed that it is closest tile)
        int maxAreaIndex = 0;
        double maxWidth = 0;
        //iterate through each rotatedRect find largest
        for (int i = 0; i < rectangle.length; i++) {
//                if(rectangle[i].size.height > 300 || rectangle[i].size.width >300) {

                        if ((double)rectangle[i].height/rectangle[i].width>1.18&&rectangle[i].width > maxWidth) {
                            maxAreaIndex = i;
                            maxWidth = rectangle[i].width;
                        }
        }
//        //if there is a detected largest contour, record information about it
        if(rectangle.length>0) {
            double ratio = abs((double)rectangle[maxAreaIndex].height/rectangle[maxAreaIndex].width);
            if(ratio>1.1) {
                poleSize = rectangle[maxAreaIndex].width;
                centerOfPole = rectangle[maxAreaIndex].x - 320;
                frameList.add(new double[]{centerOfPole, poleSize});
            }
            else{
                frameList.add(new double[] {0, 0});
            }
        }
        else{
            frameList.add(new double[] {0, 0});
        }
//        //list of frames to reduce inconsistency, not too many so that it is still real-time
        if(frameList.size()>3) {
            frameList.remove(0);
        }

        //release all the data

        input.release();

        mat.release();
        contoursPoly = null;
//        thresh5.copyTo(input);
        thresh.copyTo(input);
        thresh.release();
//        thresh2.release();
//        thresh3.release();
//        thresh4.release();
//        thresh5.release();
        hierarchy.release();
        Scalar color = new Scalar(255,0,0);
        if(rectangle.length>0) {
            Imgproc.rectangle(input, rectangle[maxAreaIndex], color, 5);
        }
        return input;
    }

    public double centerOfCone() {

        double average=0;
        for(int i=0;i<frameList.size()-1;i++){
            average+=frameList.get(i)[0];
        }
        return average/(frameList.size()-1);
    }

    public double coneSize() {
        double average=0;
        for(int i=0;i<frameList.size()-1;i++){
            if(frameList.get(i)[0]!=0&&frameList.get(i)[1]!=0) {
                average += frameList.get(i)[1];
            }
        }
        return average/(frameList.size()-1);
    }

    public double[] coneRotatedPolarCoord() {
        double consiz = coneSize();
        double center = centerOfCone();
        if(abs(center)+3 >= 320-(consiz/2.0)){
            return new double[]{0,0};
        }
        return new double[]{-atan(center/focalLength)*180/PI,abs(4.15/(2*tan(atan((center+consiz/2)/(focalLength))-atan(center/focalLength))))};
    }
}