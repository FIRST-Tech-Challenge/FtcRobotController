package org.firstinspires.ftc.team417_CENTERSTAGE.opencv;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class Constants {
    // Camera constants for OpenCV detection
    public static final int CAMERA_IMAGE_WIDTH = 640;
    public static final int CAMERA_IMAGE_HEIGHT = 480;

    // Roi (region of interest) for OpenCV detection
    public static final double xLowerBound = 0;
    public static final double xUpperBound = 1;
    public static final double yLowerBound = 2.0 / 3;
    public static final double yUpperBound = 1;
    public static final Rect roi = new Rect(0, (int) ((2 * CAMERA_IMAGE_HEIGHT) / 3.0), CAMERA_IMAGE_WIDTH, (int) (CAMERA_IMAGE_HEIGHT / 3.0));
    public static final Scalar roiColor = new Scalar(0, 255, 255); // cyan

    // Color ranges for OpenCV detection
    public static final Scalar RED_COLOR_DETECT_MAX_HSV = new Scalar(8, 255, 255);
    public static final Scalar RED_COLOR_DETECT_MIN_HSV = new Scalar(0, 60, 50);
    public static final Scalar BLUE_COLOR_DETECT_MAX_HSV = new Scalar(120, 255, 255);
    public static final Scalar BLUE_COLOR_DETECT_MIN_HSV = new Scalar(110, 60, 50);
    public static final Scalar borderColor = new Scalar(0, 255, 0);  // green

    // Sets blur size for gaussian blur in color detection
    public static final Size BLUR_SIZE = new Size(5, 5);
}
