package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //misc
    public static final double LAUNCH_HEIGHT = 0.41;
    public static final int ENCODER_TICKS_PER_REVOLUTION = 28;
    public static final double FLYWHEEL_RADIUS = 0.0765;
    public static final double GRAVITY = 9.80665;
    public static final double INCHES_PER_METER = 39.3701;
    public static final double ROBOT_RADIUS = 8.75;
    public static double ITERATIONS = 1;
    public static final double LAUNCHER_LENGTH = 0.24;
    public static final double LAUNCHER_VERTICAL_OFFSET = 0.085;
    public static final double BASE_LAUNCH_ANGLE = 19.50244851;

    public static double kpFlywheel = 0.6; //proportional constant multiplier goodish
    public static  double kiFlywheel = 1.0; //integral constant multiplier
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

    //odometry positions all in meters
    public static double goalX = 0.9144;
    public static double goalY = 3.6576;
    public static double GOAL_RADIUS = 8;
    public static double POWER_SHOT_RADIUS = 1;
    public static double startingXOffset = 1.2192;
    public static double startingYOffset = .24765;
    public static double MULTIPLIER = 1.1;


    public static int overrideTPS = 0;

    public static double  DUMMYVAL = 0.0;

    public enum Target {
        NONE(0, 0, 0),
        HIGH_GOAL(0.9144, 3.6576, 0.88),
        MID_GOAL(0.9144,3.6576,.6858),
        MID_GOAL_CLASSIC(-0.9144,3.6576,.6858),
        LOW_GOAL(0.9144,3.6576,.4318),
        FIRST_POWER_SHOT(.1016,3.6576,.8),
        SECOND_POWER_SHOT(.2921,3.6576,.8),
        THIRD_POWER_SHOT(.4826,3.6576,.8);



        public double x, y, height;

        private Target(double x, double y, double height) {
            this.x = x;
            this.y = y;
            this.height = height;
        }
    }
}
