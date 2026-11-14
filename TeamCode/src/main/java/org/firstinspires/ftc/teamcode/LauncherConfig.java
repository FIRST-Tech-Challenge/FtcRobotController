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
    public static double KP = 120.0;
    public static double KI = 0.67;
    public static double KD = 12.0;
    public static double KF = 150.0;

    public static double TARGET_VELOCITY = 1150;

    public static double AUTO_KP = 125.0;
    public static double AUTO_KI = 0.78;
    public static double AUTO_KD = 12.0;
    public static double AUTO_KF = 150.0;
    public static double AUTO_TARGET_VELOCITY = 1150;

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

    public static double INTAKE_POWER = 0.8;
}
