package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
//import org.opencv.videoio.VideoCapture;

public class  FirstVisionProcessor implements VisionProcessor {

    public Rect rectLeft = new Rect(10, 10, 200, 450);

    public Rect rectMiddle = new Rect(220, 10, 200, 450);

    public Rect rectRight = new Rect(430, 10, 200, 450);

    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    public String colorToCheck;
    //Telemetry telemetry;

    double[] colorValues = {};

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft, colorToCheck);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle, colorToCheck);
        double satRectRight = getAvgSaturation(hsvMat, rectRight, colorToCheck);
        //double satRectLeft = getAvgRedBlueSaturation(hsvMat, rectLeft, colorToCheck);
        //double satRectMiddle = getAvgRedBlueSaturation(hsvMat, rectMiddle, colorToCheck);
        //double satRectRight = getAvgRedBlueSaturation(hsvMat, rectRight, colorToCheck);

        colorValues = new double[]{satRectLeft, satRectMiddle, satRectRight};

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            return Selected.LEFT;
        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
            return Selected.MIDDLE;
        }
        return Selected.RIGHT;

       /* //Initiating the VideoCamera class (camera: : 0)
        VideoCapture Capture = new VideoCapture(0);*/




    }

    protected double getAvgSaturation(Mat input, Rect rect, String colorToCheck) {
        submat = input.submat(rect);
        int colorPosition = 1;
        Scalar color = Core.mean(submat);
        /*telemetry.addData("blue color ", color.val[0]);
        telemetry.addData("green color ", color.val[1]);
        telemetry.addData("red color ", color.val[2]);
        telemetry.update();
*/
        if (colorToCheck == "blue")
            colorPosition = 1;
        else if(colorToCheck == "red")
            colorPosition = 1;

        return color.val[colorPosition];
    }
    protected double getAvgRedBlueSaturation(Mat input, Rect rect, String colorName) {
        //bgr - default blue
        Scalar lower = new Scalar(50, 50, 100);
        Scalar upper = new Scalar(255, 255, 140);
        int colorPosition = 0;
        submat = input.submat(rect);

        if (colorName == "blue") {
            //red
            lower = new Scalar(50, 50, 100);
            upper = new Scalar(255, 255, 140);
            colorPosition = 2;
        }
        Mat colorMask = new Mat();
        Core.inRange(submat, lower, upper, colorMask);

        Scalar result = Core.mean(colorMask);
        return result.val[colorPosition];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }
    }

    public Selected getSelection() {
        return selection;
    }

    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}