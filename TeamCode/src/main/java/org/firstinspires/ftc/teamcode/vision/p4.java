package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class p4 extends OpenCvPipeline {
    public Scalar x = new Scalar(87,179,232);
    public Scalar low = new Scalar(69, 150, 200);
    public Scalar high = new Scalar(100, 200, 250);

    Mat mask, hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        mask = new Mat(input.rows(), input.cols(), CvType.CV_8UC1);

        Core.inRange(input, low, high, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        Imgproc.drawContours(input ,contours,-1, new Scalar(255,0,0));

        return input;
    }

    public String blocked(Mat input) {
        mask = new Mat(input.rows(), input.cols(), CvType.CV_8UC1);

        Core.inRange(input, low, high, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

        ArrayList<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        int numContours = 0;
        for(MatOfPoint cont : contours) {
            numContours++;
            ArrayList<Point> a = (ArrayList<Point>) cont.toList();
            int i = 0, x = 0, y = 0;
            for(Point b : a) {
                i++;
                x += b.x;
                y += b.y;
            }

        }


        return "";
    }
}
