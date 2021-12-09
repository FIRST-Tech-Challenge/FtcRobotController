package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class DuckFinder extends OpenCvPipeline {
    ArrayList<MatOfPoint> duckContours = new ArrayList<>();
    ArrayList<Point> duckCenters = new ArrayList<>();

    public ArrayList<Point> getDuckCenters() {
        return duckCenters;
    }

    // A pipeline that finds and draws contours around the the ducks in the frame
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        duckCenters.clear();
        duckContours.clear();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lower = new Scalar(22, 120, 50);
        Scalar upper = new Scalar(33, 255, 255);
//        Scalar lower = new Scalar(0, 0, 0);
//        Scalar upper = new Scalar(255, 255, 255);
        Core.inRange(mat, lower, upper, mat);
        Imgproc.GaussianBlur(mat, mat, new org.opencv.core.Size(9, 9), 0);

        Imgproc.findContours(mat, duckContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < duckContours.size(); i++) {
            Point point = new Point(duckContours.get(i).toArray()[0].x, duckContours.get(i).toArray()[0].y);
            duckCenters.add(point);
            Imgproc.drawContours(input, duckContours, i, new Scalar(255, 0, 0), 3);
            Imgproc.putText(input, "Duck", point, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0), 2);
        }
        return input;
    }
}