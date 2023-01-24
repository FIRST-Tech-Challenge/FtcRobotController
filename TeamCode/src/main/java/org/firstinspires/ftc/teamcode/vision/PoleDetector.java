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
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PoleDetector extends OpenCvPipeline
{
    Telemetry telemetry;
    public PoleDetector(Telemetry t) {telemetry = t;}
    Mat mat = new Mat();
    public Rect pole = new Rect();
    final int screenWidth = 640;

    @Override
    public Mat processFrame(Mat input)
    {
        if (input.empty()) return input;
        //return poleDetection(input, new Scalar (95,64,20), new Scalar (135,255,255));
        return poleDetection(input, new Scalar (13,64,20), new Scalar (27,255,255));

//        Logitech HD Webcam C270, 640x480
//        <Calibration
//            size="640 480"
//            focalLength="822.317f, 822.317f"
//            principalPoint="319.495f, 242.502f"
//            distortionCoefficients="-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0"
//        />
    }

    private Mat poleDetection(Mat input, Scalar lowColor, Scalar highColor) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // yellow
        Scalar lowHSV = lowColor;
        Scalar highHSV = highColor;
        Mat thresh = new Mat();

        //yellow
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.removeIf(c -> Imgproc.boundingRect(c).height < 20);
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 255, 255));

        if(!contours.isEmpty()) {
            MatOfPoint biggestPole = Collections.max(contours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
            pole = Imgproc.boundingRect(biggestPole);

            Imgproc.rectangle(input, pole, new Scalar(255, 0, 0), 2);
            Imgproc.circle(input, new Point(pole.x + (pole.width/2), pole.y + (pole.height/2)), 1, new Scalar(255, 0, 255), 3);
            Imgproc.putText(input, "Pole " + (pole.x + (pole.width/2.0)) +","+(pole.y + (pole.height/2.0)), new Point(pole.x, pole.y < 10 ? (pole.y+pole.height+20) : (pole.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
        }

        contours.clear();
        mat.release();
        edges.release();
        thresh.release();

        return input;
    }

    public double differenceX () {
        double difference = middleX() - (screenWidth/2.0);
        return difference;
    }

    public double middleX () {
        return (double) pole.x + (pole.width/2.0);
    }
}