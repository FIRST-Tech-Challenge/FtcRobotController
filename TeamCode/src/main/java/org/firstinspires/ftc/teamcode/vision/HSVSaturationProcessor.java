package org.firstinspires.ftc.teamcode.vision;
import android.graphics.Canvas;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class HSVSaturationProcessor implements VisionProcessor {
    public int nonSelectedColor = Color.GREEN; // Green
    public int selectedColor = Color.BLUE; // Blue

    Mat hsvMat = new Mat();
    Mat processedMat = new Mat();
    Mat detectionMat = new Mat();

    double leftSpikeSaturation = 0;
    double centerSpikeSaturation = 0;
    double rightSpikeSaturation = 0;

    // These are the values that we have to tune at each competition, different for BLUE_LEFT/RED_LEFT and BLUE_RIGHT/RED_RIGHT.
    public double LEFT_SPIKE_SATURATION_BASELINE = 0;
    public double CENTER_SPIKE_SATURATION_BASELINE = 0;
    public double RIGHT_SPIKE_SATURATION_BASELINE =  0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

                LEFT_SPIKE_SATURATION_BASELINE = 30; // this was 22.21125992063492 at calibration
                CENTER_SPIKE_SATURATION_BASELINE = 25; // this was 6.122269404803341 at calibration
                RIGHT_SPIKE_SATURATION_BASELINE =  0;


        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsvMat, detectionMat, 1);
        Imgproc.cvtColor(detectionMat, processedMat, Imgproc.COLOR_GRAY2RGB);

        // drawRectangles(processedMat);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,float scaleBmpPxToCanvasPx , float scaleCanvasDensity, Object userContext) {

        //draw the spike line bounding boxes on the canvas
        //drawRectangles(canvas, scaleBmpPxToCanvasPx);
//        //draw left spike box
//        Paint paint = new Paint();
//        paint.setColor(Color.GREEN);
//        paint.setStyle(Paint.Style.STROKE);
//        paint.setStrokeWidth(5);
//
//        canvas.drawRect(makeGraphicsRect(rightSpike,scaleBmpPxToCanvasPx),paint);
    }


    private static void msgOnImage(Mat input, String msg,Rect msgBoxRect) {
        // Define the font and other text properties
        int fontFace = Imgproc.FONT_HERSHEY_SIMPLEX;
        double fontScale = 1.0;
        Scalar fontColor = new Scalar(255, 255, 255); // White color
        int thickness = 2;


        // Get the size of the text
        Size textSize = Imgproc.getTextSize(msg, fontFace, fontScale, thickness, new int[]{0});

        // Calculate the position to center the text in the given rectangle
        Point textPosition = new Point(
                msgBoxRect.x + (msgBoxRect.width - textSize.width) / 2,
                msgBoxRect.y + (msgBoxRect.height + textSize.height) / 2
        );

        // Draw the text on the image
        Imgproc.putText(input,msg , textPosition, fontFace, fontScale, fontColor, thickness);
    }


    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        Mat submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    public double getLeftSpikeSaturation() {
        return leftSpikeSaturation;
    }

    public double getCenterSpikeSaturation() {
        return centerSpikeSaturation;
    }

    public double getRightSpikeSaturation() {
        return rightSpikeSaturation;
    }


}