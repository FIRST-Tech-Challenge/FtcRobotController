package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //misc
    public static final double LAUNCH_HEIGHT = 0.41;
    public static final double GOAL_HEIGHT = 0.88;
    public static final int ENCODER_TICKS_PER_REVOLUTION = 28;
    public static final double FLYWHEEL_RADIUS = 0.0765;
    public static final double GRAVITY = 9.80665;

    public static double kpFlywheel = 0.006; //proportional constant multiplier goodish
    public static  double kiFlywheel = 0.0; //integral constant multiplier
    public static  double kdFlywheel= 0.0; //derivative constant multiplier

    // Vision
    public static int TOP_LEFT_X = 224;
    public static int TOP_LEFT_Y = 303;
    public static int BOTTOM_RIGHT_X = 484;
    public static int BOTTOM_RIGHT_Y = 448;

    public static double NORMALIZE_ALPHA = 51.0;
    public static double NORMALIZE_BETA = 261.0;

    public static double BLUR_RADIUS = 8.558558558558557;

    public static double HSV_THRESHOLD_HUE_MIN = 0.4668065215846204;
    public static double HSV_THRESHOLD_HUE_MAX = 30.044343858627744;
    public static double HSV_THRESHOLD_SATURATION_MIN = 40.13039568345324;
    public static double HSV_THRESHOLD_SATURATION_MAX = 255.0;
    public static double HSV_THRESHOLD_VALUE_MIN = 109.84730100784292;
    public static double HSV_THRESHOLD_VALUE_MAX = 255.0;

    public static double MIN_CONTOUR_AREA = 1000;

    //testVars
    public static  double tempDistance= 0.0;
}
