package org.firstinspires.ftc.teamcode.Components.CV;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
@Config
public class ConeObserverPipeline extends OpenCvPipeline {
    public static double  degPerPix = 22.5/320, widTimesDist = 820*4*24/31.0, focalLength = 715;
    double centerOfPole = 0, poleSize = 0;
    ArrayList<double[]> frameList;
    public static double LowS = 50;
    public static double HighS = 255;
    public static double LowH = 100;
    public static double HighH = 140;
    public static double LowV = 0;
    public static double HighV = 255;



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


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours of edges
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
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
//
//        //find index of largest rotatedRect(assumed that it is closest tile)
        int maxAreaIndex = 0;
        double maxWidth = 0;
        //iterate through each rotatedRect find largest
        for (int i = 0; i < rectangle.length; i++) {
//                if(rectangle[i].size.height > 300 || rectangle[i].size.width >300) {
                    if (rectangle[i].size.height < rectangle[i].size.width) {
                        if (rectangle[i].size.height > maxWidth) {
                            maxAreaIndex = i;
                            maxWidth = rectangle[i].size.height;
                        }
                    } else {
                        if (rectangle[i].size.width > maxWidth) {
                            maxAreaIndex = i;
                            maxWidth = rectangle[i].size.width;
                        }
//                    }
                }
        }
//        //if there is a detected largest contour, record information about it
        if(rectangle.length>0) {
            double ratio = rectangle[maxAreaIndex].size.height/rectangle[maxAreaIndex].size.width;
            if(ratio>1.1&&ratio<1.4 ||1/ratio>1.1&&1/ratio<1.4) {

                if (rectangle[maxAreaIndex].size.height < rectangle[maxAreaIndex].size.width) {
                    poleSize = rectangle[maxAreaIndex].size.height;
                    centerOfPole = rectangle[maxAreaIndex].center.x - 320;
                } else {
                    poleSize = rectangle[maxAreaIndex].size.width;
                    centerOfPole = rectangle[maxAreaIndex].center.x - 320;

                }
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
        if(frameList.size()>4) {
            frameList.remove(0);
        }

        //release all the data

        input.release();

        mat.release();
        rectangle=null;
        contoursPoly = null;
//        masked.release();
        thresh.copyTo(input);
        thresh.release();
        hierarchy.release();

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
        if(abs(center)+5 >= 320-(consiz/2.0)||consiz/2>239){
            return new double[]{0,0};
        }
        return new double[]{-atan(center/focalLength)*180/PI,widTimesDist / consiz};
    }
}