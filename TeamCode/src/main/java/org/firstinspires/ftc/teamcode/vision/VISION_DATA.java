package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Scalar;

public class VISION_DATA {
    // Colors (Probably don't edit these)
    public static final Scalar RGB_WHITE = new Scalar(255.0, 255.0, 255.0);
    public static final Scalar RGB_BLUE = new Scalar(0.0, 0.0, 255.0);
    public static final Scalar RGB_GREEN = new Scalar(0.0, 255.0, 0.0);
    public static final Scalar RGB_RED = new Scalar(255.0, 0.0, 0.0);

    // GUI colors (used to draw boxes -> does not affect the actual image processing)
    public static final Scalar MARGIN_BOX_COLOR = RGB_WHITE;
    public static final Scalar[] BOX_COLORS = new Scalar[]{RGB_BLUE, RGB_GREEN, RGB_RED}; // Order: Left, Middle, Right

    // These values are applied after margins
    public static final int LEFT_BOX_X1 = 130;
    public static final int LEFT_BOX_Y1 = 75;
    public static final int LEFT_BOX_X2 = 220;
    public static final int LEFT_BOX_Y2 = 175;
    public static final int MIDDLE_BOX_X1 = 310;
    public static final int MIDDLE_BOX_Y1 = 75;
    public static final int MIDDLE_BOX_X2 = 395;
    public static final int MIDDLE_BOX_Y2 = 175;
    public static final int RIGHT_BOX_X1 = 480;
    public static final int RIGHT_BOX_Y1 = 75;
    public static final int RIGHT_BOX_X2 = 570;
    public static final int RIGHT_BOX_Y2 = 175;

    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 360;
    public static final int TOP_MARGIN = 0;
    public static final int LEFT_MARGIN = 0;
    public static final int RIGHT_MARGIN = 0;
    public static final int BOTTOM_MARGIN = 0;




}
