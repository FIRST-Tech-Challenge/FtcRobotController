package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;

/**
 * Live-tunable constants. {@link Config} reflects over the {@code public static} fields below,
 * so every value here shows up in FTC Dashboard.
 */
@Config
public class SwerveTeleOpConfig {

    private SwerveTeleOpConfig() {
    }

    // =========================================================
    // 1. PID COEFFICIENTS (Edit these live to tune holding power)
    // =========================================================
    public static double Kp = 0.05;
    public static double Kd = 0.0;
    public static double Ki = 0.0; // Usually 0 for drive modules
    public static double Kf = 0.0;
    public static double Kl = 0.0; // Integral limit

    // =========================================================
    // 2. MODULE OFFSETS (Values from your Zeroing OpMode)
    // =========================================================
    // Enter the raw angles you read when wheels are pointing FORWARD
    public static double module1Adjust = 35.0;
    public static double module2Adjust = 65.0;
    public static double module3Adjust = -140.0;

    // =========================================================
    // 3. DRIVER PREFERENCES
    // =========================================================
    public static double DRIVE_SPEED_SCALAR = 0.8;
    public static double ROTATION_SPEED_SCALAR = 0.8;
    public static boolean FIELD_CENTRIC = true;
    public static boolean USE_IMU = true;

    // Set to -1 if rotating the robot clockwise makes heading DECREASE
    // Defaulting to -1 to match the GoBilda Pinpoint yaw direction used in this bot.
    public static double IMU_POLARITY = -1.0;

    // =========================================================
    // 4. KINEMATICS / GEOMETRY
    // =========================================================
    // Effective radius of rotation. Increase this if the robot spins too fast/slow relative to drive.
    // In your old code this was implicitly "1.0".
    public static double ROBOT_RADIUS = 1.0;

    // Example: if facing driver and translation is mirrored, try +90 or -90.
    public static double HEADING_FRAME_OFFSET_DEG = 0.0;

    public static double HEADING_LOCK_KP = 0.017;
    public static double HEADING_LOCK_KI = 0.00000;
    public static double HEADING_LOCK_KD = 0.0001;
    public static double HEADING_LOCK_KF = 0.00000;
    public static double HEADING_LOCK_KL = 0.0;
    public static double HEADING_LOCK_DEADBAND = 0.2;
    public static double P = 0.0008;
}
