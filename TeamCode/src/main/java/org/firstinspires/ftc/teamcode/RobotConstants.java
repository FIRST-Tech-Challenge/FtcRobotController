package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class RobotConstants {
    @Config
    public static class LiftConfig {
        public static double P = 0.0195;
        public static double I = 0.0;
        public static double D = 0.0002;
        public static double F = 0.0;

        public static int ZERO = 0;
        public static int LOW = 737;
        public static int MID = 1170;
        public static int HIGH = 1600;

        public static double MANUAL_ADJUSTMENT_MULTI = 50;
    }

    @Config
    public static class WristConfig {
        public static double FORWARDS  = 0.8275;
        public static double BACKWARDS = 0.171;
        public static double REST      = 0.5;
    }
}
