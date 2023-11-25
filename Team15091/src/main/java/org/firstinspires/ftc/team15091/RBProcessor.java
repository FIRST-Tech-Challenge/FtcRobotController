package org.firstinspires.ftc.team15091;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.List;

public class RBProcessor implements VisionProcessor {

    private static final Paint yellowPaint = new Paint();
    private static final Paint greenPaint = new Paint();

    String debug = "";
    Scalar[] means = new Scalar[2];
    public PixelPosition position = PixelPosition.Right;

    // Define region for left and middle
    static final List<Point> circleCenters = Arrays.asList(
            new Point(30, 315),
            new Point(360, 285)
    );
    static final int radius = 25;
    public void init(int width, int height, CameraCalibration cameraCalibration) {
        yellowPaint.setColor(Color.YELLOW);
        yellowPaint.setStyle(Paint.Style.STROKE);
        yellowPaint.setStrokeWidth(2);
        greenPaint.setColor(Color.GREEN);
        greenPaint.setStyle(Paint.Style.FILL);
    }

    public Object processFrame(Mat input, long captureTimeNanos) {
        return input;
    }

    private boolean isRed(double hueValue) {
        return (hueValue > 0 && hueValue < 30) || hueValue > 150;
    }

    private boolean isBlue(double hueValue) {
        return hueValue > 90 && hueValue < 150;
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object input) {
        Mat temp = new Mat();

        // Convert to RBG to HSV, so we can extract HUE channel
        Imgproc.cvtColor((Mat)input, temp, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(temp, temp, Imgproc.COLOR_RGB2HSV);

        for (int i = 0; i < 2; i++) {
            means[i] = calculateMeanHSVInCircle(temp, circleCenters.get(i), radius);
        }
        temp.release();

        if (isRed(means[0].val[0]) || isBlue(means[0].val[0]) && means[0].val[1] > 70) { // add an additional saturation check
            position = PixelPosition.Left;
            canvas.drawCircle((float)circleCenters.get(0).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(0).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, greenPaint);
        } else if (isRed(means[1].val[0]) || isBlue(means[1].val[0]) && means[1].val[1] > 70) {
            position = PixelPosition.Middle;
            canvas.drawCircle((float)circleCenters.get(1).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(1).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, greenPaint);
        } else {
            position = PixelPosition.Right;
        }
        debug = String.format(" %3.0f, %3.0f", means[0].val[0], means[1].val[1]);

        canvas.drawCircle((float)circleCenters.get(0).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(0).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, yellowPaint);
        canvas.drawCircle((float)circleCenters.get(1).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(1).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, yellowPaint);
    }

    private static Scalar calculateMeanHSVInCircle(Mat image, Point center, int radius) {
        Mat mask = new Mat(image.size(), CvType.CV_8U, Scalar.all(0));
        Imgproc.circle(mask, center, radius, new Scalar(255), -1);  // Draw a filled circle on the mask
        Scalar meanColor = meanColorInCircle(image, mask);
        mask.release();  // Release resources
        return meanColor;
    }

    private static Scalar meanColorInCircle(Mat image, Mat mask) {
        // Apply the mask to the image
        Mat maskedImage = new Mat();
        Core.bitwise_and(image, image, maskedImage, mask);

        // Calculate the mean color using Core.mean
        Scalar meanColor = Core.mean(maskedImage, mask);

        // Release resources
        maskedImage.release();

        return meanColor;
    }
}