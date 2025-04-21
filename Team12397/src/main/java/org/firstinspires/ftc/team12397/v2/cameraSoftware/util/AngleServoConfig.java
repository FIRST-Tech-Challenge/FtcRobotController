package org.firstinspires.ftc.team12397.v2.cameraSoftware.util;

import com.acmerobotics.dashboard.config.Config;

/** All servo‑angle tuning parameters live in one place and are FTC‑Dashboard‑bindable */
@Config
public final class AngleServoConfig {

    public static double SERVO_CENTER         = 0.505;     // re‑measured
    public static double ANGLE_TO_POS_GAIN    = 1.0 / 360; // calibrated (≈0.00278)

    public static double MOTION_THRESHOLD_POS      = 0.003; // stop when <0.3 % travel
    public static double MAX_DELTA_POS_PER_UPDATE  = 0.10;  // ≤¼ travel per 10 ms

    public static long   UPDATE_INTERVAL_MS   = 10;
    public static double SMOOTHING_ALPHA      = 0.25;      // smoother error, still responsive
    public static double MIN_POS              = 0.10;
    public static double MAX_POS              = 0.90;

    private AngleServoConfig() {}
}
