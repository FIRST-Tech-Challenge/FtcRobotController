package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvPipeline;


public class DetectMarker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DetectionPipeline pipeline = new DetectionPipeline(telemetry, "red");

    }
}

public enum allianceColor {
    RED,
    BLUE
}

class DetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    public enum MarkerLocation {
        Left,
        Middle,
        Right,
        Not_Found
    }

    public enum allianceColor { // maybe we can enumerate it?
        RED,                    // Inserting it here because its the only place allianceColor seems to be referenced at.
        BLUE,
        UNIDENTIFIED
    }

    String allianceColor; // TODO: Standardize allianceColor
    private MarkerLocation markerLocation;


    static final Rect LEFT_RECT = new Rect(
            new Point(60, 35),
            new Point(110, 75));

    static final Rect MIDDLE_RECT = new Rect(
            new Point(110, 35),
            new Point(150, 75));

    static final Rect RIGHT_RECT = new Rect(
            new Point(150, 35),
            new Point(200, 75));

    static double PERCENT_COLOR_THRESHOLD = 0.4;


    Mat mat = new Mat();

    public DetectionPipeline(Telemetry t, String ac) {
        telemetry = t;
        allianceColor = ac; // ac stands for allianceColor
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // TODO: Change COLOR_RGB2HSV to something more useful.
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


        if (markerLeft) {
            markerLocation = MarkerLocation.Left;
            telemetry.addData("Marker Location", "right");
        } else if (markerMiddle) {
            markerLocation = MarkerLocation.Middle;
            telemetry.addData("Marker Location", "middle");
        } else if (markerRight) {
            markerLocation = MarkerLocation.Right;
            telemetry.addData("Marker Location", "left");
        } else {
            markerLocation = MarkerLocation.Not_Found;
            telemetry.addData("Marker Location", "not found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB); // TODO: Change COLOR_GRAY2RGB to something more useful.

        Scalar colorNormal;

        if (allianceColor.equals("red")) {
            colorNormal = new Scalar(255, 0, 0);
        }
        else {
            colorNormal = new Scalar(0, 0, 255);
        }

        Scalar colorMarker = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_RECT, markerLocation == MarkerLocation.Left ? colorMarker : colorNormal);
        Imgproc.rectangle(mat, MIDDLE_RECT, markerLocation == MarkerLocation.Middle ? colorMarker : colorNormal);
        Imgproc.rectangle(mat, RIGHT_RECT, markerLocation == MarkerLocation.Right ? colorMarker : colorNormal);

        return mat;
    }
}
