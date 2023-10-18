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

public class Draw3RectanglesProcessor implements VisionProcessor {

    // currently using MOKOSE 4K webcam - overkill for FTC work
    public Rect rectLeft = new Rect(50, 50, 150, 300);  // OpenCV camera coordinates
    public Rect rectCenter = new Rect(250, 50, 150, 300);
    public Rect rectRight = new Rect(450, 50, 150, 300);
    Selected selection = Selected.NONE;

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
     * @param scaleBmpPxToCanvasPx dimensions
     * @return android rectangle
     */
    private android.graphics.Rect makeGraphicsRect(
            Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    /**
     * setup the rectPaint variable
     * @param canvas canvas
     * @param onscreenWidth width
     * @param onscreenHeight height
     * @param scaleBmpPxToCanvasPx conversion
     * @param scaleCanvasDensity scale
     * @param userContext user
     */
    @Override
    public void onDrawFrame(Canvas canvas,  int  onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);  // easier to view with factor of 4

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(
                rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleCenter = makeGraphicsRect(
                rectCenter, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(
                rectRight, scaleBmpPxToCanvasPx);

        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case CENTER:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleCenter, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }
    }

    public enum Selected {NONE, LEFT, CENTER, RIGHT}
}
