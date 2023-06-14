package org.firstinspires.ftc.teamcode.Components.CV;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.tan;

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
public class StickObserverPipeline extends OpenCvPipeline {
//    public static double  degPerPix = 22.5/320, widTimesDist = 820*12/16.0*14.7/11.5*9.5/9, focalLength = 715;
    double centerOfPole = 0, poleSize = 0, contourSize = 0.0;
    boolean poleInView = false;
    double[] contourDimensions = {0,0};
    ArrayList<double[]> frameList;
    public static double LowS = 120;
    public static double HighS = 255;
    public static double LowH = 20;
    public static double HighH = 30;
    public static double LowV = 100;
    public static double HighV = 255;
    public static double minWidth = 20;
    public static double minAreaThresh = 0.1;



    public StickObserverPipeline() {
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

//        Mat masked = new Mat();
//        //color the white portion of thresh in with HSV from mat
//        //output into masked
//        Core.bitwise_and(mat, mat, masked, thresh);
//        //calculate average HSV values of the white thresh values
//        Scalar average = Core.mean(masked,thresh);
//
//        Mat scaledMask = new Mat();
//        //scale the average saturation to 150
//        masked.convertTo(scaledMask,-1,150/average.val[1],0);
//
//
//        Mat scaledThresh = new Mat();
//        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
//        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
//        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
//        Core.inRange(scaledMask, strictLowHSV,strictHighHSV,scaledThresh);
//
//        Mat finalMask = new Mat();
//        //color in scaledThresh with HSV(for showing result)
//        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours of edges
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        //rotatedRect because it allows for more accurate bounding rectangles, good if pole is slanted
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
//            if(rectangle[i].size.height/rectangle[i].size.width>2 ||rectangle[i].size.width/rectangle[i].size.height>2) {
                if(rectangle[i].size.height > 200 || rectangle[i].size.width >200) {
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
                    }
                }
//            }
        }
//        //if there is a detected largest contour, record information about it
        if(rectangle.length>0) {
            if(rectangle[maxAreaIndex].size.height<rectangle[maxAreaIndex].size.width) {
                poleSize = rectangle[maxAreaIndex].size.height;
                centerOfPole = rectangle[maxAreaIndex].center.x - 320;
                if(rectangle[maxAreaIndex].size.width>480.0*0.8/*&&poleSize>minWidth*/){
                    poleInView = true;
                }
                else{
                    poleInView = false;
                }
                contourDimensions = new double[]{poleSize, rectangle[maxAreaIndex].size.width};

            }
            else{
                poleSize = rectangle[maxAreaIndex].size.width;
                centerOfPole = rectangle[maxAreaIndex].center.x - 320;
                if(rectangle[maxAreaIndex].size.height>480.0*0.8/*&&poleSize>minWidth*/){
                    poleInView = true;
                }
                else{
                    poleInView = false;
                }
                contourDimensions = new double[]{poleSize, rectangle[maxAreaIndex].size.height};
            }
            frameList.add(new double[]{centerOfPole, poleSize});
        }
        else{
            frameList.add(new double[] {0, 0});
        }

//        //list of frames to reduce inconsistency, not too many so that it is still real-time
        if(frameList.size()>2) {
            frameList.remove(0);
        }
        contourSize = Core.sumElems(thresh).val[0]*0.00083;
        if(contourSize>minAreaThresh){
            poleInView=true;
        }
//        else if (!poleInView&&contourDimensions[0]==0&&contourDimensions[1]==0 && contourSize<minAreaThresh){
//            poleInView= false;
//        }

        //release all the data

        input.release();
//        scaledThresh.copyTo(input);
//        scaledThresh.release();
//        scaledMask.release();
        mat.release();
        rectangle=null;
        contoursPoly = null;
//        masked.release();
        thresh.copyTo(input);
        thresh.release();
        hierarchy.release();
//        finalMask.release();
//        Scalar lineColor= new Scalar(255,50,50);
//        if(contoursPoly.length>0) {
//            Imgproc.rectangle(input, Imgproc.boundingRect(contoursPoly[maxAreaIndex]), lineColor, 5);
//        }
        return input;
    }

    public double[] getContourDimensions(){
        return contourDimensions;
    }

   public double centerOfPole() {
        //256.227,257.307,252.9,253.414: 4,11.75|| 18.8 .073
        //2.5,12.75: 162.7,161.6, 161.7,162.5||  11.09 .068
        //2,9.5 : 187.45, 187.26|| 11.88  .0648
         //1,8,7.8 : 273 || 12.99 .047
        //10.6,22.2 :
        //4.1,20.6 :
        double average=0;
        for(int i=0;i<frameList.size()-1;i++){
            average+=frameList.get(i)[0];
        }
        return average/(frameList.size()-1);
    }

    public double poleSize() {
        double average=0;
        for(int i=0;i<frameList.size()-1;i++){
            if(frameList.get(i)[0]!=0&&frameList.get(i)[1]!=0) {
                average += frameList.get(i)[1];
            }
        }
        return average/(frameList.size()-1);
    }
    public double getContourSize(){
        return contourSize;
    }

    public double[] poleRotatedPolarCoord() {
        double consiz = poleSize;
        double center = centerOfPole;
        if(abs(center)+5 >= 320-(consiz*0.5)||consiz*0.5>239){
            return new double[]{0,0};
        }
        return new double[]{1.1*-atan(center*.0014)*57.296, abs(1.3/(2*tan(atan((center+consiz*0.5)*.0014)-atan(center*.0014))))};
    }
}