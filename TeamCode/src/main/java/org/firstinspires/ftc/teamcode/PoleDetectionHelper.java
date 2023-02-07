package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

//for dashboard
/*@Config*/
public class PoleDetectionHelper extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;

    Telemetry telemetry;

    public PoleDetectionHelper(Telemetry t) {
        frameList = new ArrayList<>();
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        /**
         * for (MatOfPoint contour : contours){
         * MatOfPoint2f name_of_Var = new MatOfPoint2f(contour.toArray());
         * RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);
         *
         * Do some reading on above functions
         *
         *
         * define area
         * double rectangleArea = boundingRect.size.area();
         * int rectCount = 0;
         *
         * if(rectangleArea > minimum && rectangleArea < max){//Threshold for what size to consider correct target
         *      rectCount++;
         *     Point[] rotated_rect_points = new Point[4];
         *     final Scalar BLUE = new Scalar(0, 0, 255);
         *     boundingRect.points(rotated_rect_points); //Populate points
         *
         *     Rect currRect = Imgproc.boundingRect(new MatOfPoint(rotated_rect_points));\\Creates rectangle object that we can draw
         *     //Another good function to look up and read about
         *
         *     Imgproc.rectangle(input,currRect,BLUE,2);
         *     //Passes the current image, current Rectangle, color to draw, and line thickness
         *     Imgproc.putTex(input, "Rectangle: " + rectCount, currRect.br(), 1, 1, BLUE);
         *
         *
         *     }
         * //Something to think about
         * //Store rectangles in array and create method to return array when called so you can do math with the rectangles
         * }
         *
         *
         */


        double minimum = 0.0;
        double maximum = 1000000.0;


        for (MatOfPoint contour : contours){
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);
            //Do some reading on above functions
            //define area
            double rectangleArea = boundingRect.size.area();
            int rectCount = 0;
            if(rectangleArea > minimum && rectangleArea < maximum){//Threshold for what size to consider correct target
                rectCount++;
                Point[] rotated_rect_points = new Point[4];
                Point point1 = new Point(50, 50);
                Point point2 = new Point(200, 200);
                final Scalar BLUE = new Scalar(0, 0, 255);
                boundingRect.points(rotated_rect_points); //Populate points
                Rect currRect = Imgproc.boundingRect(new MatOfPoint(rotated_rect_points));//Creates rectangle object that we can draw
                //Another good function to look up and read about
                //Imgproc.rectangle(input,currRect,BLUE,2);
                Imgproc.rectangle(input, point1, point2, BLUE);
                //Passes the current image, current Rectangle, color to draw, and line thickness
                Imgproc.putText(mat, "Rectangle: " + rectCount, currRect.br(), 1, 1, BLUE);//change mat to input
            }
            //Something to think about
            //Store rectangles in array and create method to return array when called so you can do math with the rectangles

        }


        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }


        //release all the data
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
       // mat.release();
        masked.release();
        edges.release();
        thresh.release();
        finalMask.release();
        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return mat;
    }


}
