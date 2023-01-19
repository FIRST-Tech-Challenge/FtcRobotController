package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PoleDetector extends OpenCvPipeline
{
    Telemetry telemetry;
    public PoleDetector(Telemetry t) {telemetry = t;}
    Mat mat = new Mat();
    public Rect maxRect = new Rect();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty()) return input;

        // yellow
        Scalar yellow_lowHSV = new Scalar (10,64,20);
        Scalar yellow_highHSV = new Scalar (30,255,255);
        Mat thresh = new Mat();

        //yellow
        Core.inRange(mat, yellow_lowHSV, yellow_highHSV, thresh);

        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 200);

//        Logitech HD Webcam C270, 640x480
//        <Calibration
//            size="640 480"
//            focalLength="822.317f, 822.317f"
//            principalPoint="319.495f, 242.502f"
//            distortionCoefficients="-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0"
//        />

        List<MatOfPoint> initialContours = new ArrayList<>();
        List<MatOfPoint> finalContours = new ArrayList<>();
        MatOfInt hull = new MatOfInt();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, initialContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < initialContours.size(); i++) {
            Imgproc.convexHull(initialContours.get(i), hull);
            if(hull.total() >= 10){
                finalContours.add(initialContours.get(i));
                Imgproc.putText(mat, String.valueOf(hull.total()), initialContours.get(i).toArray()[0], 1, 1, new Scalar(0, 255, 0));
                Imgproc.drawContours(mat, initialContours, i, new Scalar(255, 255, 255), 2);
            }
        }

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[finalContours.size()];
        Rect[] boundRect = new Rect[finalContours.size()];
        for (int j = 0; j < finalContours.size(); j++) {
            contoursPoly[j] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(finalContours.get(j).toArray()), contoursPoly[j], 3, true);
            boundRect[j] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[j].toArray()));
            if(boundRect[j].area() > maxRect.area()) maxRect = boundRect[j];
            Imgproc.rectangle(mat, boundRect[j], new Scalar(255, 255, 255));
        }
        Imgproc.rectangle(mat, maxRect, new Scalar(255, 0, 0), 2);
        Imgproc.circle(mat, new Point(maxRect.x + (maxRect.width/2), maxRect.y + (maxRect.height/2)), 1, new Scalar(255, 0, 255), 3);

        telemetry.addData("maxRect", maxRect.area());
        telemetry.update();

        return mat;
    }

    public int differenceX () {
        int difference = middleX() - (mat.width()/2);
        return difference;
    }

    public int middleX () {
        return maxRect.x + (maxRect.width/2);
    }
}