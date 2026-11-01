package org.firstinspires.ftc.teamcode.scoring;

public final class ScoringConfig {
    private ScoringConfig() {
        // Utility class
    }

    public static final double TRIGGER_THRESHOLD = 0.1;
    public static boolean intake_side_and_stratight_combined = true;

    public static final double INTAKE_POWER = 1.0;
    public static final double WINDMILL_POWER = 0.75;
    public static final double WINDMILL_INTAKE_ASSIST_POWER = 0.5;

    public static final double SHOOTER_BASELINE_RPM = 3100.0;

    public static final double SHOOTER_YAW_ADJUSTER = 100;
    public static final double SHOOTER_YAW_ADJUST_THRESHOLD_DEGREES = 20.0;

    public static final double SHOOTER_STEP_RPM = 100.0;
    public static final double SHOOTER_MIN_RPM = 0.0;
    public static final double SHOOTER_MAX_RPM = 5600.0;
    public static final double SHOOTER_RPM_TOLERANCE = 100.0;
    public static final double SHOOTER_TICKS_PER_REV = 28.0;
}


