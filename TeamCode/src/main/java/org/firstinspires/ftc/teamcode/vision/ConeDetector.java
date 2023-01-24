package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ConeDetector extends OpenCvPipeline
{
    Telemetry telemetry;
    public ConeDetector(Telemetry t) {telemetry = t;}
    Mat mat = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty()) return input;
        /* blue
        Scalar blue_lowHSV = new Scalar (95,64,20);
        Scalar blue_highHSV = new Scalar (135,255,255);
        Mat thresh = new Mat();
        */

        //red
        Scalar red1_lowHSV = new Scalar (0, 64, 20);
        Scalar red1_highHSV = new Scalar (20, 255, 255);
        Mat thresh1 = new Mat();

        Scalar red2_lowHSV = new Scalar(160, 64, 20);
        Scalar red2_highHSV = new Scalar (180, 255, 255);
        Mat thresh2 = new Mat();
        //red finish

        //blue
        //Core.inRange(mat, blue_lowHSV, blue_highHSV, thresh);

        //red
        Core.inRange(mat, red1_lowHSV, red1_highHSV, thresh1);
        Core.inRange(mat, red2_lowHSV, red2_highHSV, thresh2);
        Mat thresh = new Mat();
        Core.bitwise_or(thresh1, thresh2, thresh);
        //red finish

        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();
        MatOfInt hull = new MatOfInt();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            Imgproc.convexHull(contours.get(i), hull);
            if(hull.total() >= 10 && hull.total() <= 50){
                Imgproc.drawContours(mat, contours, i, new Scalar(255, 255, 255), 2);
                Imgproc.putText(mat, String.valueOf(hull.total()), contours.get(i).toArray()[0], 1, 1, new Scalar(0, 255, 0));
            }
        }

        /*
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }
        for (int i = 0; i != boundRect.length; i++) Imgproc.rectangle(mat, boundRect[i], new Scalar(255, 255, 255));


         */

        return mat;
    }
}