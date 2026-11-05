package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/*
 * Launcher values you can change LIVE from FTC Dashboard (http://192.168.43.1:8080/dash).
 * They must stay 'public static' and NOT 'final' for Dashboard to edit them.
 * Once a value feels right, write it down here so it survives a restart.
 *
 * Tuning order: kF first, then kP, then kD/kI only if needed.
 */
@Config
public class LauncherConfig {
    // Tuned and tested on the robot (SuperTestTeleop bench test).
    public static double KP = 120.0;
    public static double KI = 0.2;
    public static double KD = 12.0;
    public static double KF = 150.0;

    public static double RPM_TOLERANCE = 50; // "ready" when within this many RPM
    public static double DEFAULT_RPM = 1200; // tested working speed

    public static double FEEDER_REST_POS = 0.3; // held while not feeding
    public static double FEEDER_FEED_POS = 0.04; // held while the feed trigger is pressed
}
