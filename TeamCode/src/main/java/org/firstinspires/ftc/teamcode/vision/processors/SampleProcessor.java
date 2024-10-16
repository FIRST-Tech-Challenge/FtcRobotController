package org.firstinspires.ftc.teamcode.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleProcessor extends OpenCvPipeline implements VisionProcessor {


    private Telemetry telemetry;
    public Scalar redLowerBound = new Scalar(0, 96, 87);
    public Scalar redUpperBound = new Scalar(90, 255, 255);
    public Scalar blueLowerBound = new Scalar(48, 113, 162);
    public Scalar blueUpperBound = new Scalar(114, 255, 255);


    Mat temp = new Mat();
    Mat red = new Mat();
    Mat blue = new Mat();
    Mat yellow = new Mat();

    Mat kernel = Mat.ones(3, 3, CvType.CV_32F);

    ArrayList<Rect> redRects = new ArrayList<>();
    ArrayList<Rect> blueRects = new ArrayList<>();
    ArrayList<Rect> yellowRects = new ArrayList<>();
    List<MatOfPoint> cont = new ArrayList<>();


    public SampleProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, temp, Imgproc.COLOR_RGB2HSV);

        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_ERODE, kernel, new Point(0, 0), 3);
        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_DILATE, kernel, new Point(0, 0), 4);

        Core.inRange(temp, redLowerBound, redUpperBound, red);
        Core.inRange(temp, blueLowerBound, blueUpperBound, blue);

        List<MatOfPoint> contours = new ArrayList<>();
        cont = contours;

        findBoundingBoxes(red, redRects, contours);
        findBoundingBoxes(blue, blueRects, contours);
        findBoundingBoxes(yellow, yellowRects, contours);

        return frame;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);

        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_ERODE, kernel, new Point(0, 0), 3);
        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_DILATE, kernel, new Point(0, 0), 4);

        Core.inRange(temp, redLowerBound, redUpperBound, red);
        Core.inRange(temp, blueLowerBound, blueUpperBound, blue);

        List<MatOfPoint> contours = new ArrayList<>();
        findBoundingBoxes(red, redRects, contours);
        findBoundingBoxes(blue, blueRects, contours);

        return input;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);
        int minSize = 2000;
        paint.setColor(Color.MAGENTA);

        drawBoundingBoxes(canvas, paint, scaleBmpPxToCanvasPx, redRects, minSize);

        paint.setColor(Color.CYAN);
        drawBoundingBoxes(canvas, paint, scaleBmpPxToCanvasPx, blueRects, minSize);

        paint.setColor(Color.YELLOW);
        drawBoundingBoxes(canvas, paint, scaleBmpPxToCanvasPx, yellowRects, minSize);

        telemetry.update();

    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }


    private void findBoundingBoxes(Mat mat, ArrayList<Rect> rects, List<MatOfPoint> contourList) {
        Imgproc.findContours(mat, contourList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        rects.clear();
        for (int i = 0; i < contourList.size(); i++) {

            MatOfPoint point = contourList.get(i);
            List<Point> list = point.toList();
            for (int j = 0; j < list.size(); j++) {
                telemetry.addData("contours " + i, "X: " + list.get(j).x + " Y: " + list.get(j).y);
            }
            Rect boundingRect = Imgproc.boundingRect(point);
            rects.add(boundingRect);
        }
        telemetry.addData("Count", contourList.size());
        telemetry.update();




        contourList.clear();
    }

    private void drawBoundingBoxes(Canvas canvas, Paint paint, float scaleBmpPxToCanvasPx,
                                   ArrayList<Rect> rects, int minSize) {
        for (int i = 0; i < rects.size(); i++) {
            if (rects.get(i) != null) {
                android.graphics.Rect rect = makeGraphicsRect(rects.get(i), scaleBmpPxToCanvasPx);
                if (rect.width() * rect.height() > minSize)
                    canvas.drawRect(rect, paint);

            }
        }
    }

}
