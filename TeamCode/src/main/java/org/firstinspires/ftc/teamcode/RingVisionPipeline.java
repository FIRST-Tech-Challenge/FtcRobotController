package org.firstinspires.ftc.teamcode;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;

public class RingVisionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat mat2 = new Mat();
    Date date = new Date();
    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    public enum Location {
        C_FULL_STACK,
        B_HALF_STACK,
        A_NO_STACK
    }
    private Location location;

    private int hueMin = 10;
    private int hueMax = 26;
    private int satMin = 116;
    private int satMax = 255;
    private int valMin = 120; //68
    private int valMax = 255;
    private ArrayList<Double> ringAreaArray;
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public RingVisionPipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        //converts frame to HSV colorspace
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //the range of colors to filter.
        Scalar lowHSV = new Scalar(hueMin, satMin, valMin);
        Scalar highHSV = new Scalar(hueMax, satMax, valMax);

        Core.inRange(mat, lowHSV, highHSV, mat);
        telemetry.addData("HueMin: ", hueMin);
        telemetry.addData("HueMax: ", hueMin);
        telemetry.addData("ValMin: ", valMin);
        telemetry.addData("ValMax: ", valMax);
        telemetry.addData("SatMin: ", satMin);
        telemetry.addData("SatMax: ", satMax);



        Imgproc.findContours(mat, contours, mat2, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        ringAreaArray = getContourArea(mat);
        telemetry.addData("timer", date.getTime());
        telemetry.addData("Contor area: ", getContourArea(mat));
        telemetry.update();

        return mat;
    }

    public Location getLocation() {
        return location;
    }



    private ArrayList<Double> getContourArea(Mat mat) {
        Mat hierarchy = new Mat();
        Mat image = mat.clone();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(image, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<Double> arr = new ArrayList<Double>();
        double minArea = 300;
        for (int i = 0; i < contours.size(); i++) {
            Mat contour = contours.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if(contourArea > minArea){
                arr.add(contourArea);
            }
        }
        location = getRingArea();
        return arr;
    }

    private Location getRingArea() {
        if (ringAreaArray == null) {
            return Location.A_NO_STACK;
        }
        double biggestRingArea = 0;
        for (int i = 0; i < ringAreaArray.size(); i++) {
            if(ringAreaArray.get(i) > biggestRingArea) {
                biggestRingArea = ringAreaArray.get(i);
            }
        }
        //Change these numbers for size determining
        Location size = Location.A_NO_STACK;
        if (biggestRingArea > 1300) {
            size = Location.C_FULL_STACK;
        } else if (biggestRingArea > 300) {
            size = Location.B_HALF_STACK;
        } else {
            size = Location.A_NO_STACK;
        }

        return size;
    }

}