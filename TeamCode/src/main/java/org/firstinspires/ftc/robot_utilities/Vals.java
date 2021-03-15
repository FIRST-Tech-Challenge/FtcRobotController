package org.firstinspires.ftc.robot_utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Vals {
    public static double flywheel_kp = 40;
    public static double flywheel_ki = 0.1;
    public static double flywheel_kd = 0;
    public static double flywheel_ks = 0;
    public static double flywheel_kv = 0.03;
    public static double flywheel_tolerance = 5;
    public static double flywheel_speed = 1125;
    public static double flywheel_direction = -1;
    public static int flywheel_ready_ticks = 13;
    public static int flywheel_ready_min_speed = 1120;
    public static int flywheel_ready_max_speed = 1140;
    public static double flywheel_filtered_speed = 0;
    public static int flywheel_max_achievable_ticks = 1360;

    public static double rotate_kp = .015;
    public static double rotate_ki = .11;
    public static double rotate_kd = .0003;
    public static double rotate_tolerance = 1;
    public static double rotate_target = 0;

    public static final double TICKS_PER_INCH_MOVEMENT = 40.58;

    public static double hitter_start = 0.4;
    public static double hitter_end = .7;

}
