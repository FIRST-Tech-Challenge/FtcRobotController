package org.firstinspires.ftc.team6220_CENTERSTAGE;

import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class Constants {

    // Color ranges for OpenCV detection
    public final int[] RED_COLOR_DETECT_MAX_HSV = {10, 255, 255};
    public final int[] RED_COLOR_DETECT_MIN_HSV = {0, 65, 95};
    public final int[] BLUE_COLOR_DETECT_MAX_HSV = {140, 255, 255};
    public final int[] BLUE_COLOR_DETECT_MIN_HSV = {90, 65, 25};

    // Sets blur size for gaussian blur in color detection
    public static final Size BLUR_SIZE = new Size(5, 5);

}
