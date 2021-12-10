package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class DuckFinder extends OpenCvPipeline {
    ArrayList<MatOfPoint> duckContours = new ArrayList<>();
    ArrayList<Point> duckCenters = new ArrayList<>();
    int index0 = 0;
    boolean duckOnScreen = false;

    public ArrayList<Point> getDuckCenters() {
        return duckCenters;
    }

    public Point getFirstCenter() {
        if(duckOnScreen && duckCenters.size() > index0) {
            return duckCenters.get(index0);
        } else {
            return null;
        }
    }

    // A pipeline that finds and draws contours around the the ducks in the frame
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lower = new Scalar(22, 60, 50);
        Scalar upper = new Scalar(33, 255, 255);
//        Scalar lower = new Scalar(0, 0, 0);
//        Scalar upper = new Scalar(255, 255, 255);
        Core.inRange(mat, lower, upper, mat);
        Imgproc.GaussianBlur(mat, mat, new org.opencv.core.Size(9, 9), 0);

        duckContours.clear();
        Imgproc.findContours(mat, duckContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        if(duckContours.size() > 0) {
            index0 = duckCenters.size();
            duckOnScreen = true;
        } else {
            duckOnScreen = false;
        }
        for (int i = 0; i < duckContours.size(); i++) {
            Point point = new Point(duckContours.get(i).toArray()[0].x, duckContours.get(i).toArray()[0].y);
            duckCenters.add(point);
            Imgproc.drawContours(input, duckContours, i, new Scalar(255, 0, 0), 3);
            Imgproc.putText(input, "Duck " + duckContours.get(i).toArray()[0].x, point, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0), 2);
        }
        mat.release();
        return input;
    }
}