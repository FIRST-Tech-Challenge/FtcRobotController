package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static final double LAUNCH_HEIGHT = 0.41;
    public static final double GOAL_HEIGHT = 0.88;
    public static final int ENCODER_TICKS_PER_REVOLUTION = 28;
    public static final double FLYWHEEL_RADIUS = 0.0765;
    public static final double GRAVITY = 9.80665;

    public static double kpFlywheel = 0.006; //proportional constant multiplier goodish
    public static  double kiFlywheel = 0.0; //integral constant multiplier
    public static  double kdFlywheel= 0.0; //derivative constant multiplier
}
