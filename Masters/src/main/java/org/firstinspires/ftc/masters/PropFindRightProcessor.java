package org.firstinspires.ftc.masters;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.text.TextPaint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PropFindRightProcessor implements VisionProcessor, CameraStreamSource {

    public  Rect interestMid = new Rect(35, 185, 32, 50);
    public  Rect interestRight = new Rect(320, 260, 32, 50);

    private final Scalar upper = new Scalar(0,50,0); // lower bounds for masking
    private final Scalar lower = new Scalar(255,255,255); // upper bounds for masking
    private TextPaint textPaint = null;
    private Paint linePaint = null;


    public enum pos {
        LEFT,
        MID,
        RIGHT,
    }

    public PropFindRightProcessor.pos position = PropFindRightProcessor.pos.LEFT;

    Telemetry telemetry;
    TelemetryPacket packet;


    Mat HSV = new Mat();
    Mat H = new Mat();
    Mat S = new Mat();
    Mat V = new Mat();

    Mat region_h_mid = new Mat();
    Mat region_s_mid = new Mat();
    Mat region_h_right = new Mat();
    Mat region_s_right = new Mat();

    int avg_h_mid = 0;
    int avg_s_mid = 0;
    int avg_h_right = 0;
    int avg_s_right = 0;

    Mat mask = new Mat(), diff_im = new Mat();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    void inputToHSV(Mat input) {
        Imgproc.cvtColor(input,HSV,Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, H, 0);
        Core.extractChannel(HSV, S, 1);
        Core.extractChannel(HSV, V, 2);
    }


    public PropFindRightProcessor(Telemetry telemetry, TelemetryPacket packet) {
        this.telemetry = telemetry;
        this.packet = packet;

        // setting up the paint for the text in the center of the box
        textPaint = new TextPaint();
        textPaint.setColor(Color.GREEN); // you may want to change this
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setAntiAlias(true);
        textPaint.setTextSize(40); // or this
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);

        // setting up the paint for the lines that comprise the box
        linePaint = new Paint();
        linePaint.setColor(Color.GREEN); // you may want to change this
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(10); // or this
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeJoin(Paint.Join.ROUND);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
//        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        // this method comes with all VisionProcessors, we just don't need to do anything here, and you dont need to call it
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        inputToHSV(input);

        Core.inRange(HSV, new Scalar(0,50,0), new Scalar(255,255,255), mask);

        diff_im = new Mat();
        Core.add(diff_im, Scalar.all(0), diff_im);
        Core.bitwise_not(mask,mask);
        input.copyTo(diff_im, mask);
        diff_im.copyTo(input);
        diff_im.release();

        inputToHSV(input);

        region_h_mid = H.submat(interestMid);
        region_s_mid = S.submat(interestMid);
        region_h_right = H.submat(interestRight);
        region_s_right = S.submat(interestRight);

        avg_h_mid = (int) Core.mean(region_h_mid).val[0];
        avg_s_mid = (int) Core.mean(region_s_mid).val[0];
        avg_h_right = (int) Core.mean(region_h_right).val[0];
        avg_s_right = (int) Core.mean(region_s_right).val[0];

        if (avg_s_mid <5) {
            position = PropFindRightProcessor.pos.MID;
        } else if (avg_s_right <5) {
            position = PropFindRightProcessor.pos.RIGHT;
        } else {
            position = PropFindRightProcessor.pos.LEFT;
        }

        telemetry.addData("position", position);
        telemetry.update();

        Imgproc.rectangle(input, new Point(interestMid.x, interestMid.y), new Point(interestMid.x + interestMid.width, interestMid.y+ interestMid.height), new Scalar(0,255,0),1 );
        Imgproc.rectangle(input, new Point(interestRight.x, interestRight.y), new Point(interestRight.x + interestRight.width, interestRight.y+ interestRight.height), new Scalar(0,255,0),1 );

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // this method draws the rectangle around the largest contour and puts the current prop position into that rectangle
        // you don't need to call it

//		for (MatOfPoint contour : contours) {
//			Rect rect = Imgproc.boundingRect(contour);
//			canvas.drawLines(new float[]{rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx}, textPaint);
//		}

//        Imgproc.rectangle(input, new Point(interestMid.x, interestMid.y), new Point(interestMid.x + interestMid.width, interestMid.y+ interestMid.height), new Scalar(0,255,0),1 );
//        Imgproc.rectangle(input, new Point(interestRight.x, interestRight.y), new Point(interestRight.x + interestRight.width, interestRight.y+ interestRight.height), new Scalar(0,255,0),1 );
//
//        // if the contour exists, draw a rectangle around it and put its position in the middle of the rectangle
//        if (largestContour != null) {
//            Rect rect = Imgproc.boundingRect(largestContour);
//
//            float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};
//
//            canvas.drawLine(points[0], points[1], points[0], points[3], linePaint);
//            canvas.drawLine(points[0], points[1], points[2], points[1], linePaint);
//
//            canvas.drawLine(points[0], points[3], points[2], points[3], linePaint);
//            canvas.drawLine(points[2], points[1], points[2], points[3], linePaint);
//
//            String text = String.format(Locale.ENGLISH, "%s", recordedPropPosition.toString());
//
//            canvas.drawText(text, (float) largestContourX * scaleBmpPxToCanvasPx, (float) largestContourY * scaleBmpPxToCanvasPx, textPaint);
        }

    public void close() {
//        hierarchy.release();
//        sel1.release();
//        sel2.release();
    }

    @Override
    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

    }
}