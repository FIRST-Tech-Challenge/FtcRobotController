package org.firstinspires.ftc.teamcode.Robot.AutoDrive;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.TimeUnit;

/**
 * @brief custom vision processor using FTC EZ OpenCV libraries
 * adapted from Learn Java for FTC by Alan G Smith
 * @link https://raw.githubusercontent.com/alan412/LearnJavaForFTC/master/LearnJavaForFTC.pdf
 *
 * methods: https://javadoc.io/doc/org.firstinspires.ftc/Vision/latest/org/firstinspires/ftc/vision/VisionProcessor.html
 *  void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration)
 *  void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
 *  Object processFrame(org.opencv.core.Mat frame, long captureTimeNanos)
 */

public class AutoPreLoadDeliveries implements VisionProcessor {

    // currently using MOKOSE 4K webcam - overkill for FTC work
    private VisionPortal visionPortal = null;        // Used to manage the video source
    // for use on display screen as visual feedback during Team Prop detection in CENTERSTAGE
    public Rect rectLeft = new Rect(50, 100, 150, 300);  // OpenCV camera coordinates
    public Rect rectCenter = new Rect(250, 100, 150, 300);
    public Rect rectRight = new Rect(450, 100, 150, 300);
    Selected selection = Selected.NONE;
    // for image processing
    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    /**
     * initialization of the OpMode
     * @param width in pixels
     * @param height in pixels
     * @param calibration camera calibration parameters
     */
    public void init(int width, int height, CameraCalibration calibration) {
        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(500L, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(20);
    }

    @Override
    /**
     * process a single frame of the captured image stream
     * @param frame the matrix
     * @param captureTimeNanos capture time in nanoseconds
     */
    public Object processFrame(Mat frame, long captureTimeNanos) {
        /*
         * convert the colorspace
         * from RGB (Red, green, blue) to HSV (Hue, Saturation, Value).
         * This operation is useful for performing image detection in FTC
         * since the background is the gray from the mat tiles.
         * The Hue is the shade of color (useful when we are looking for something specific),
         * the Saturation is how much of the color there is (so gray has a low saturation), and
         * the Value is how bright it is. (This is sometimes called Brightness which means HSB)
         */
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        // obtain the average saturation of each rectangle with a helper function
        double satRectLeft   = getAvgSaturation(hsvMat, rectLeft);
        double satRectCenter = getAvgSaturation(hsvMat, rectCenter);
        double satRectRight  = getAvgSaturation(hsvMat, rectRight);

        /*
         * determine which of the three rectangles (LEFT, CENTER, RIGHT) has the
         * highest saturation, in other words, the least amount of gray
         * underlying assumption is that Team Prop (either blue or red) has greater
         * saturation than the background mat tiles
         */
        if ((satRectLeft > satRectCenter) && (satRectLeft > satRectRight)) {
            return Selected.LEFT;
        } else if ((satRectCenter > satRectLeft) && (satRectCenter > satRectRight)) {
            return  Selected.CENTER;
        }
        return Selected.RIGHT;
    }

    /**
     * helper function to estimate the saturation value of a rectangle in a frame
     * @param input
     * @param rect
     * @return
     * - create a sub-matrix from the image frame based on the rectangle of interest
     * - obtain the average (or mean) for the pixels in the rectangle
     */
    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];  // 0 for Hue, 1 for Saturation and 2 for Value
    }

    /**
     * convert from OpenCV camera rectangle to an android.graphics.Rect entity in screen coordinates
     * @param rect
     * @param scaleBmpPxToCanvasPx
     * @return
     */
    private android.graphics.Rect makeGraphicsRect(
            Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top  = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    /**
     * setup the rectPaint variable
     * @param canvas
     * @param onscreenWidth
     * @param onscreenHeight
     * @param scaleBmpPxToCanvasPx
     * @param scaleCanvasDensity
     * @param userContext
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

        selection = (Selected) userContext;
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

    // the OpMode can get the selection to make the decision on the Team Prop position
    public Selected getSelection() {
        return selection;
    }

    public enum Selected {NONE, LEFT, CENTER, RIGHT}
}
