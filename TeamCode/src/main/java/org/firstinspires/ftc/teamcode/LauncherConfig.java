package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/*
 * Launcher + intake values you can change live from FTC Dashboard.
 * Must stay 'public static' (not 'final'). Dashboard changes are lost on restart.
 * Flywheel speeds are in ticks/sec (what setVelocity() and getVelocity() use).
 */
@Config
public class LauncherConfig {
    // teleop flywheel
    public static double KP = 28.0;
    public static double KI = 1;
    public static double KD = 0.0;
    public static double KF = 13.0;

    public static double TARGET_VELOCITY = 1920;



    public static double AUTO_KP = 28.0;
    public static double AUTO_KI = 1.0;
    public static double AUTO_KD = 0.0;
    public static double AUTO_KF = 13.0;
    public static double AUTO_TARGET_VELOCITY = 1920;

    public static double VELOCITY_TOLERANCE = 50;
    public static double REVERSE_POWER = 0.5;

    public static double FEEDER_REST_POS = 0.3;
    public static double FEEDER_FEED_POS = 0.04;

    // seconds
    public static double READY_TIMEOUT = 2.0;
    public static double FEED_HOLD_TIME = 0.3;
    public static double FEED_PAUSE_TIME = 0.5;
    public static double SCAN_TIME = 8.0;
    public static double PARK_TIME = 8.0;

    public static double INTAKE_POWER = 0.35;
}
