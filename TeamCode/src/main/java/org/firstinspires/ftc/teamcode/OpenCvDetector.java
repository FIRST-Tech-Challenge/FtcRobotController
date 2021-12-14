package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

// https://gist.github.com/oakrc/12a7b5223df0cb55d7c1288ce96a6ab7.
// Thank you Team Wolf Corp (#12525)
public class OpenCvDetector extends OpenCvPipeline {

    static final int REGION_WIDTH = 35;
    static final int REGION_HEIGHT = 25;

    enum ElementLocation {
        LEFT,
        RIGHT,
        MIDDLE
    }

    private int width = 1280; // width of the image
    ElementLocation location;


    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking
        Log.w("OpenCv Pipeline","Starting frame proccessing.");
        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

//        // if something is wrong, we assume there's no skystone
//        if (mat.empty()) {
//            location = ElementLocation.NONE;
//            return input;
//        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow

        Mat thresh = new Mat();
        // Declare where the three points you are looking at are
        Point LEFT_POINT = new Point(50,98);
        Point MIDDLE_POINT = new Point(100,98);
        Point RIGHT_POINT = new Point(150,98);
        // Find things in our yellow range and put them in thresh
        Core.inRange(mat, lowHSV, highHSV, thresh);
        // Use canny edge detection to find edges of threshold objects
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);
        // Use findCountours to smooth it out
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        // Parse said contours to make boxes
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }
        for (int i = 0; i != boundRect.length; i++) {
            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8),3);
            Log.i("Item Location", String.valueOf(boundRect[i]));
        }


        Log.w("OpenCv Pipeline","Returning");
        Mat output = new Mat();
        Imgproc.cvtColor(mat, output, Imgproc.COLOR_HSV2RGB);
        return output;
    }

    public ElementLocation getLocation() {
        return this.location;
    }
}