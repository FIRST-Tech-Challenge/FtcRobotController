package org.firstinspires.ftc.teamcode.robots.goodBot.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //misc

    public static int visionView = 0;
    public static final int ENCODER_TICKS_PER_REVOLUTION = 560;
    public static final double INCHES_PER_METER = 39.3700787;
    public static Alliance ALLIANCE = Alliance.RED;
    public static int ALLIANCE_INT_MOD = 1;
    public static boolean isInner = false;

    // Vision
    public static int TOP_LEFT_X_RED = 70;
    public static int TOP_LEFT_Y_RED = 160;
    public static int BOTTOM_RIGHT_X_RED = 180;
    public static int BOTTOM_RIGHT_Y_RED = 230;

    public static int TOP_LEFT_X_BLUE = 70;
    public static int TOP_LEFT_Y_BLUE = 160;
    public static int BOTTOM_RIGHT_X_BLUE = 250;
    public static int BOTTOM_RIGHT_Y_BLUE = 230;

    public static double NORMALIZE_ALPHA = 51.0;
    public static double NORMALIZE_BETA = 261.0;

    public static double BLUR_RADIUS = 8.558558558558557;

    public static double HSV_THRESHOLD_HUE_MIN = 0.4668065215846204;
    public static double HSV_THRESHOLD_HUE_MAX = 1000;
    public static double HSV_THRESHOLD_SATURATION_MIN = 64;
    public static double HSV_THRESHOLD_SATURATION_MAX = 255.0;
    public static double HSV_THRESHOLD_VALUE_MIN = 109.84730100784292;
    public static double HSV_THRESHOLD_VALUE_MAX = 255.0;

    public static double MIN_CONTOUR_AREA = .1;

    public static double MIN_BLOB_SIZE = 4000;
    public static double VISION_ONE_TO_FOUR_ASPECT = 1.5;

    public static int WEBCAM_WIDTH = 320;
    public static int WEBCAM_HEIGHT = 240;

    public static double startingXOffset = 0;
    public static double startingYOffset = 0;

    public enum Alliance {
        RED,
        BLUE;
    }

    public enum Position {
        //headings and elevations that are negative means don't apply them to ending position - let other behaviors control
        START(49/INCHES_PER_METER, 49/INCHES_PER_METER,0, 0),
        START_INNER((49-24)/INCHES_PER_METER, 49/INCHES_PER_METER,0, 0),
        HOME(0, 0,0, 0);


        public double x, y, baseHeading, launchHeading, launchElevation, launchStart;

        public double getX(){return x * ALLIANCE_INT_MOD;}

        private Position(double x, double y, double baseHeading, double launchStart) {
            this.x = x;
            this.y = y;
            this.baseHeading=baseHeading; //-1 means take no action on changing
            this.launchStart=launchStart;  //number 0.0 to 1.0 - progression of travel where launcher actions should start
        }
    }
}