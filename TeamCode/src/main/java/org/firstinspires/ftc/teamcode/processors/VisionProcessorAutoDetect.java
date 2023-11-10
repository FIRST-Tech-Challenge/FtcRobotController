package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class VisionProcessorAutoDetect implements VisionProcessor {
    public Rect rectLeft = new Rect(8, 120, 100, 200);
    public Rect rectMiddle = new Rect(160, 70, 200, 100);
    public Rect rectRight = new Rect(500, 120, 100, 200);
    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Define the lower and upper bounds for red color in HSV
        Scalar lowerRed = new Scalar(0, 100, 100);
        Scalar upperRed = new Scalar(10, 255, 255);

        // Create a mask for the red regions
        Mat mask = new Mat();
        Core.inRange(hsvMat, lowerRed, upperRed, mask);

        // Find contours in the mask to create connected components
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Calculate the sizes of connected components
        List<Double> regionSizes = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            regionSizes.add(area);
        }

        // Sort the region sizes in descending order
        Collections.sort(regionSizes, Collections.reverseOrder());

        // Choose the three largest region sizes
        List<Double> largestRegionSizes = regionSizes.subList(0, Math.min(3, regionSizes.size()));

        // Determine the position of the largest region
        int imageCenterX = frame.cols() / 2;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (largestRegionSizes.contains(area)) {
                Rect boundingRect = Imgproc.boundingRect(contour);
                int regionCenterX = boundingRect.x + boundingRect.width / 2;
                String position;
                if (regionCenterX < imageCenterX - 50) {
                    return Selected.LEFT;
                } else if (regionCenterX > imageCenterX + 50) {
                    return Selected.RIGHT;
                } else {
                    return Selected.MIDDLE;
                }
            }
        }
        return Selected.NONE;
    }


    private android.graphics.Rect makeGraphicsRect(Rect rect, float  scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx); int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom); }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint(); selectedPaint.setColor(Color.RED); selectedPaint.setStyle(Paint.Style.STROKE); selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
        Paint nonSelectedPaint = new Paint(selectedPaint); nonSelectedPaint.setColor(Color.GREEN);
        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft,  scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle,  scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight,  scaleBmpPxToCanvasPx);
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

    public Selected getSelection() { return selection;
    }

    public enum Selected {
        NONE,
        LEFT, MIDDLE, RIGHT
    }
}
