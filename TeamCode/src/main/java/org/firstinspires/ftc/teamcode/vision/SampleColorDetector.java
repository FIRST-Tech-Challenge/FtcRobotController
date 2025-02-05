package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.SampleColorDetector.SampleColor.RED;
import static org.firstinspires.ftc.teamcode.constants.Constants.VisionConstants.*;
import static org.firstinspires.ftc.teamcode.vision.SampleColorDetector.SampleColor.*;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * OpenCV pipeline to detect the prop in FTC 2023 - 2024 Centerstage
 */
public class SampleColorDetector extends OpenCvPipeline {
    public enum SampleColor {
        RED,
        BLUE,
        YELLOW
    }

    private Telemetry telemetry;

    private final Mat hsvMat          = new Mat(),
                      threshold0      = new Mat(),
                      threshold1      = new Mat(),
                      hierarchy       = new Mat(),
                      cvErodeKernel   = new Mat(),
                      thresholdOutput = new Mat(),
                      erodeOutput     = new Mat();

    public SampleColorDetector(@NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        detectSample(input, RED);

        return input;
    }

    private void detectSample(Mat input, SampleColor color) {
        switch (color) {
            case RED:
                // Check if the image is in range, then adds the ranges together
                Core.inRange(hsvMat, LOW_HSV_RANGE_RED_ONE, HIGH_HSV_RANGE_RED_ONE, threshold0);
                Core.inRange(hsvMat, LOW_HSV_RANGE_RED_TWO, HIGH_HSV_RANGE_RED_TWO, threshold1);
                Core.add(threshold0, threshold1, thresholdOutput);
                break;
            case BLUE:
                // Checks if the image is in range
                Core.inRange(hsvMat, LOW_HSV_RANGE_BLUE, HIGH_HSV_RANGE_BLUE, thresholdOutput);
                break;
            case YELLOW:
                Core.inRange(hsvMat, LOW_HSV_RANGE_YELLOW, HIGH_HSV_RANGE_YELLOW, thresholdOutput);
                break;
        }

        // Erode to remove noise
        Imgproc.erode(
                thresholdOutput,
                erodeOutput,
                cvErodeKernel,
                CV_ANCHOR,
                ERODE_PASSES,
                CV_BORDER_TYPE,
                CV_BORDER_VALUE
        );

        // Finds the contours of the image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                erodeOutput,
                contours,
                hierarchy,
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        // Creates bounding rectangles along all of the detected contours
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        for (Rect rect : boundRect) {
            Imgproc.rectangle(input, rect, BOUNDING_RECTANGLE_COLOR);
        }

        telemetry.update();
    }
}