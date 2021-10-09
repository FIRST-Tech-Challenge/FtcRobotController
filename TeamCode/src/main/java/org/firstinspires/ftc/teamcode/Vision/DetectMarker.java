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

    static double PERCENT_COLOR_THRESHOLD = 0.4;


    Mat mat = new Mat();
    public DetectMarker(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_RECT);
        Mat middle = mat.submat(MIDDLE_RECT);
        Mat right = mat.submat(RIGHT_RECT);

        double leftValue = Core.sumElems(left).val[0] / LEFT_RECT.area() / 255;
        double middleValue = Core.sumElems(left).val[0] / MIDDLE_RECT.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_RECT.area() / 255;

        left.release();
        middle.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean markerLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean markerMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean markerRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (markerLeft && markerMiddle && markerRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Marker Location", "not found");
        }
        else if (markerRight) {
            location = Location.RIGHT;
            telemetry.addData("Marker Location", "right");
        }
        else if (markerLeft) {
            location = Location.LEFT;
            telemetry.addData("Marker Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;
    }
}
