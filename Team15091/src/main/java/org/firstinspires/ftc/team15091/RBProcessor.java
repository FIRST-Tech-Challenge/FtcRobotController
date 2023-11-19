package org.firstinspires.ftc.team15091;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class RBProcessor implements VisionProcessor {

    private static final Paint yellowPaint = new Paint();
    private static final Paint greenPaint = new Paint();

    Mat temp = new Mat(), sub;
    String debug = "";
    Double[] hues = new Double[2];
    public PixelPosition position = PixelPosition.Right;

    // Define region for left and middle
    static final Rect maskLeft = new Rect(0, 190, 60, 50);
    static final Rect maskMiddle = new Rect(320, 165, 60, 50);

    public void init(int width, int height, CameraCalibration cameraCalibration) {
        yellowPaint.setColor(Color.YELLOW);
        yellowPaint.setStyle(Paint.Style.STROKE);
        yellowPaint.setStrokeWidth(2);
        greenPaint.setColor(Color.GREEN);
        greenPaint.setStyle(Paint.Style.FILL);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    public Object processFrame(Mat input, long captureTimeNanos) {
        // Convert to RBG to HSV, so we can extract HUE channel
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(temp, temp, Imgproc.COLOR_RGB2HSV);
//        Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);

        sub = temp.submat(maskLeft);
        hues[0] = Core.mean(sub).val[0];
        sub.release();

        sub = temp.submat(maskMiddle);
        hues[1] = Core.mean(sub).val[0];
        sub.release();
        temp.release();

        if (isRed(hues[0]) || isBlue(hues[0])) {
            position = PixelPosition.Left;
        } else if (isRed(hues[1]) || isBlue(hues[1])) {
            position = PixelPosition.Middle;
        } else {
            position = PixelPosition.Right;
        }
        debug = String.format(" %3.0f, %3.0f", hues[0], hues[1]);

        return position;
    }

    private boolean isRed(double hueValue) {
        return hueValue > 0 && hueValue < 20;
    }

    private boolean isBlue(double hueValue) {
        return hueValue > 99 && hueValue < 120;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object lastPosition) {
        canvas.drawRect(makeGraphicsRect(maskLeft, scaleBmpPxToCanvasPx), yellowPaint);
        canvas.drawRect(makeGraphicsRect(maskMiddle, scaleBmpPxToCanvasPx), yellowPaint);
        if (lastPosition == PixelPosition.Left) {
            canvas.drawRect(makeGraphicsRect(maskLeft, scaleBmpPxToCanvasPx), greenPaint);
        }
        else if (lastPosition == PixelPosition.Middle) {
            canvas.drawRect(makeGraphicsRect(maskMiddle, scaleBmpPxToCanvasPx), greenPaint);
        }
    }
}