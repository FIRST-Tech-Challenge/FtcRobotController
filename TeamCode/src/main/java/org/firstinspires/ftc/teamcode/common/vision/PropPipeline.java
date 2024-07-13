package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.enums.PropDirection;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class PropPipeline implements VisionProcessor {
    Scalar leftColor, centerColor, rightColor;
    double leftDistance, centerDistance = 1;
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLACK = new Scalar(0, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static Scalar targetColor = RED;
    private volatile PropDirection direction = PropDirection.CENTER;
    static final Point LEFT_REGION_TOP_LEFT_POINT = new Point(45, 260);
    static final Point CENTER_REGION_TOP_LEFT_POINT = new Point(225, 240);
    static final Point RIGHT_REGION_TOP_LEFT_POINT = new Point(600, 260);
    static final int LEFT_REGION_WIDTH = 20;
    static final int LEFT_REGION_HEIGHT = 20;
    static final int CENTER_REGION_WIDTH = 20;
    static final int CENTER_REGION_HEIGHT = 20;
    static final int RIGHT_REGION_WIDTH = 20;
    static final int RIGHT_REGION_HEIGHT = 20;
    LinearOpMode opMode = null;

    public PropPipeline(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    static final Point leftRegionPointA = new Point(
            LEFT_REGION_TOP_LEFT_POINT.x,
            LEFT_REGION_TOP_LEFT_POINT.y);
    static final Point leftRegionPointB = new Point(
            LEFT_REGION_TOP_LEFT_POINT.x + LEFT_REGION_WIDTH,
            LEFT_REGION_TOP_LEFT_POINT.y + LEFT_REGION_HEIGHT);
    static final Point centerRegionPointA = new Point(
            CENTER_REGION_TOP_LEFT_POINT.x,
            CENTER_REGION_TOP_LEFT_POINT.y);
    static final Point centerRegionPointB = new Point(
            CENTER_REGION_TOP_LEFT_POINT.x + CENTER_REGION_WIDTH,
            CENTER_REGION_TOP_LEFT_POINT.y + CENTER_REGION_HEIGHT);

    static final Point rightRegionPointA = new Point(
            RIGHT_REGION_TOP_LEFT_POINT.x,
            RIGHT_REGION_TOP_LEFT_POINT.y);
    static final Point rightRegionPointB = new Point(
            RIGHT_REGION_TOP_LEFT_POINT.x + RIGHT_REGION_WIDTH,
            RIGHT_REGION_TOP_LEFT_POINT.y + RIGHT_REGION_HEIGHT);
    static Mat leftRegion, centerRegion, rightRegion = new Mat();
    static Rect leftRect, centerRect, rightRect;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        double leftColorDistance = 0;
        double centerColorDistance = 0;
        double rightColorDistance = 0;

        double leftChannelValue = 0;
        double centerChannelValue = 0;
        double rightChannelValue = 0;

        leftRect = new Rect(leftRegionPointA, leftRegionPointB);
        centerRect = new Rect(centerRegionPointA, centerRegionPointB);
        rightRect = new Rect(rightRegionPointA, rightRegionPointB);

        leftRegion = input.submat(leftRect);
        centerRegion = input.submat(centerRect);
        rightRegion = input.submat(rightRect);

        Imgproc.rectangle(input, leftRegionPointA, leftRegionPointB, BLACK, 2);
        Imgproc.rectangle(input, centerRegionPointA, centerRegionPointB, BLACK, 2);
        Imgproc.rectangle(input, rightRegionPointA, rightRegionPointB, BLACK, 2);

        leftColor = Core.mean(leftRegion);
        centerColor = Core.mean(centerRegion);
        rightColor = Core.mean(rightRegion);

        leftChannelValue = leftColor.val[2];
        centerChannelValue = centerColor.val[2];
        rightChannelValue = rightColor.val[2];

//        opMode.telemetry.addData("left stdDev: ", colorVariation(leftColor));
//        opMode.telemetry.addData("center stdDev: ", colorVariation(centerColor));
//        opMode.telemetry.addData("right stdDev: ", colorVariation(rightColor));
//        opMode.telemetry.update();

        double leftVar = colorVariation(leftColor);
        double centerVar = colorVariation(centerColor);
        double rightVar = colorVariation(rightColor);

        if ((rightVar < 15) && centerVar < 15) {
            direction = PropDirection.LEFT;
            leftRegion.setTo(GREEN);
        } else if (rightVar > centerVar) {
            direction = PropDirection.RIGHT;
            rightRegion.setTo(GREEN);
        } else {
            direction = PropDirection.CENTER;
            centerRegion.setTo(GREEN);
        }
        return (input);
    }

    public double colorVariation(Scalar color) {
        double r = color.val[0];
        double g = color.val[1];
        double b = color.val[2];

        double mean = (r + g + b) / 3.0;
        double stdDev = (Math.abs(mean - r) + Math.abs(mean - g) + Math.abs(mean - b)) / 3.0;

        return (stdDev);
    }

    public double colorDistance(Scalar color1, Scalar color2) {
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        double r2 = color2.val[0];
        double g2 = color2.val[1];
        double b2 = color2.val[2];

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public PropDirection getPropDirection() {
        return this.direction;
    }
}