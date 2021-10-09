package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class DetectMarker extends OpenCvPipeline {
    Telemetry telemetry;
    public enum MarkerLocation {
        Left,
        Middle,
        Right
    }
    private MarkerLocation markerLocation;

    static final Rect LEFT_RECT = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect MIDDLE_RECT = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_RECT = new Rect(
            new Point(140, 35),
            new Point(200, 75));

    Mat mat = new Mat();
    public DetectMarker(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(23, 50, 70);
        return null;
    }
}
