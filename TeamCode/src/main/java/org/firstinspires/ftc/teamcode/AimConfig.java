package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/*
 * Auto-aim values you can change live from FTC Dashboard (calibrated in AimTestOdom).
 * Must stay 'public static' (not 'final'). Dashboard changes are lost on restart.
 */
@Config
public class AimConfig {
    // Limelight tilt servo
    public static double LIMELIGHT_SERVO_POS = 0.3;

    // Forward/back: drive until the tag's ty (plus AXIAL_ADJUST) is zero
    public static double AXIAL_ADJUST = 2.15;
    public static double AXIAL_GAIN = 0.03;
    public static double MAX_AXIAL = 0.5;
    public static double AXIAL_DEADBAND = 1.0; // degrees

    // Turn: P on tx, plus a damping term from the Pinpoint turn rate
    public static double YAW_GAIN = 0.025;
    public static double ROTATION_VELOCITY_GAIN = 0.12;
    public static double MAX_YAW = 0.5;
    public static double YAW_DEADBAND = 0.7; // degrees

    // Smoothing: a reading's weight is (1 - SMOOTHING_ALPHA)^age, over the last 10 loops
    public static double SMOOTHING_ALPHA = 0.2;

    // "Aimed" when both drive powers are below this: the TeleOp starts intake + feed
    public static double AIMED_POWER = 0.05;
}
