package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class DuckFinder extends OpenCvPipeline {
    ArrayList<MatOfPoint> duckContours = new ArrayList<>();
    // A pipeline that finds and draws rectangles around the the ducks in the frame
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(input, input, new org.opencv.core.Size(10, 10), 0);
        Scalar lower = new Scalar(22, 120, 50);
        Scalar upper = new Scalar(33, 255, 255);
        Core.inRange(input, lower, upper, input);
        Imgproc.findContours(input, duckContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < duckContours.size(); i++) {
            Imgproc.drawContours(input, duckContours, i, new Scalar(255, 0, 0), 3);
            Imgproc.rectangle(input, Imgproc.boundingRect(duckContours.get(i)).tl(), Imgproc.boundingRect(duckContours.get(i)).br(), new Scalar(0, 255, 0), 3);
        }
        return input;
    }
}
