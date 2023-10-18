package org.firstinspires.ftc.teamcode.Concept;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

/**
 * draw a rectangle on the display medium
 *
 * @link https://raw.githubusercontent.com/alan412/LearnJavaForFTC/master/LearnJavaForFTC.pdf
 */

public class DrawRectangleProcessor implements VisionProcessor {

    public Rect rect = new Rect(50, 50, 150, 300);  // OpenCV camera coordinates

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    /**
     * convert from OpenCV camera rectangle to an android.graphics.Rect entity in screen coordinates
     * @param rect rectangle dimensions
     * @param scaleBmpPxToCanvasPx scale
     * @return android rectangle
     */
    private android.graphics.Rect makeGraphicsRect(
            Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    /**
     * setup the rectPaint variable
     * @param canvas canvas
     * @param onscreenWidth         on
     * @param onscreenHeight on
     * @param scaleBmpPxToCanvasPx scale
     * @param scaleCanvasDensity scale
     * @param userContext users
     */
    @Override
    public void onDrawFrame(Canvas canvas,  int  onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.RED);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);  // easier to view with factor of 4

        canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
    }
}
