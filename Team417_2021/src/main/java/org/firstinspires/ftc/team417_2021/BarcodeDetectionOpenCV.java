package org.firstinspires.ftc.team417_2021;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BarcodeDetectionOpenCV extends OpenCvPipeline {

    Mat frame = new Mat();
    Mat yuv = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();
    Mat blurred = new Mat();

    double maxArea = Double.MIN_VALUE;
    int barcodeIndex = 0;

    List<MatOfPoint> contours = new ArrayList<>();

    //                         y, u, v
    Scalar lower = new Scalar (0, 62, 0);
    Scalar higher = new Scalar (255, 117, 116);

    Rect rect = new Rect();
    Rect maxRect = new Rect();

    int x;
    int index;

    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(frame);

        Imgproc.cvtColor(input, yuv, Imgproc.COLOR_RGB2YUV);
        maxArea = 0.0;

        Imgproc.GaussianBlur(yuv, blurred, new Size(13,13), 0);
        Core.inRange(blurred, lower, higher, mask);

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            rect = Imgproc.boundingRect(contour);
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxRect = rect;
                maxArea = area;
                x = maxRect.x;
            }
        }
        contours.clear();

        index = findBarcodeIndex(x);

        return input;
    }

    // takes x coordinate of largest green contour in order to see where the element is set
    public int findBarcodeIndex(int x) {
        // 213, 426
        int index = 0;
        if (x < 163) {
            index = 0;
        } else if (x > 163 && x < 376) {
            index = 1;
        } else if (x > 376) {
            index = 2;
        }
        return index;
    }

}
