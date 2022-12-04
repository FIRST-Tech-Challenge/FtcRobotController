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

public class PoleDetector extends OpenCvPipeline
{
    Telemetry telemetry;
    public PoleDetector(Telemetry t) {telemetry = t;}
    Mat mat = new Mat();

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

        List<MatOfPoint> contours = new ArrayList<>();
        MatOfInt hull = new MatOfInt();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            Imgproc.convexHull(contours.get(i), hull);
            if(hull.total() >= 8){
                Imgproc.drawContours(mat, contours, i, new Scalar(255, 255, 255), 2);

                /*
                MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
                Rect[] boundRect = new Rect[contours.size()];
                for (int j = 0; j < contours.size(); j++) {
                    contoursPoly[j] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(j).toArray()), contoursPoly[j], 3, true);
                    boundRect[j] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[j].toArray()));
                }
                for (int j = 0; j != boundRect.length; j++) Imgproc.rectangle(mat, boundRect[j], new Scalar(255, 255, 255));
                */

                Imgproc.putText(mat, String.valueOf(hull.total()), contours.get(i).toArray()[0], 1, 1, new Scalar(0, 255, 0));
            }
        }

        return mat;
    }
}