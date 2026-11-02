package org.firstinspires.ftc.teamcode.navigate;

/**
 * Central place for drive/odometry configuration values that are safe to tune quickly.
 *
 * Keep PID gains in their controller classes unless a mentor explicitly wants those exposed.
 */
public final class DriveConfig {

    private DriveConfig() {
    }

    // Snap headings for trigger shortcuts.
    public static final double RIGHT_TRIGGER_TARGET_RADIANS = Math.toRadians(270.0);
    public static final double LEFT_TRIGGER_TARGET_RADIANS = Math.toRadians(90.0);

    // Snap-turn non-PID behavior limits and thresholds.
    public static final double TURN_STOP_DEADBAND_RADIANS = Math.toRadians(1.0);
    public static final double SETTLE_TOLERANCE_RADIANS = Math.toRadians(2.0);
    public static final double BRAKE_ZONE_RADIANS = Math.toRadians(45.0);
    public static final double FINE_CONTROL_ZONE_RADIANS = Math.toRadians(6.0);
    public static final double SNAP_TURN_MAX_POWER = 0.70;
    public static final double SNAP_TURN_BRAKE_MAX_POWER = 0.70;
    public static final double SNAP_TURN_SETTLE_MAX_POWER = 0.17;
    public static final double SNAP_TURN_MIN_POWER = 0.14;
    public static final double SNAP_TURN_BRAKE_MIN_POWER = 0.10;
    public static final double MAX_SETTLED_TURN_RATE_RADIANS = Math.toRadians(5.0);
    public static final int REQUIRED_SETTLED_LOOPS = 7;
    public static final double TRIGGER_THRESHOLD = 0.5;
    public static final double MANUAL_ROTATE_DEADBAND = 0.05;
    public static final double SNAP_TURN_MAX_ERROR_RATE_RADIANS_PER_SECOND = Math.toRadians(300.0);
    public static final double SNAP_TURN_ERROR_RATE_FILTER_ALPHA = 0.72;
    public static final double SNAP_TURN_MAX_COMMAND_SLEW_PER_SECOND = 3.2;

    // Driver-tunable speed scaling for translation/rotation commands in TeleOp.
    public static final double DEFAULT_DRIVE_SPEED_SCALE = 1.0;
    public static final double MIN_DRIVE_SPEED_SCALE = 0.5;
    public static final double MAX_DRIVE_SPEED_SCALE = 1.0;
    public static final double DRIVE_SPEED_SCALE_STEP = 0.1;
    public static final double DRIVE_STICK_DEADBAND = 0.06;

    // Target-navigation non-PID behavior limits.
    public static final double TARGET_POSITION_TOLERANCE_INCHES = 1.5;
    public static final double TARGET_HEADING_TOLERANCE_RADIANS = Math.toRadians(3.0);
    public static final double TARGET_MAX_TRANSLATION_COMMAND = 0.65;
    public static final double TARGET_MIN_TRANSLATION_COMMAND = 0.12;
    public static final double TARGET_MAX_ROTATION_COMMAND = 0.55;
    public static final double TARGET_MIN_ROTATION_COMMAND = 0.10;

    // Odometry geometry/field defaults.
    public static final double FIELD_SIZE_FEET = 12.0;
    public static final double FIELD_SIZE_INCHES = FIELD_SIZE_FEET * 12.0;
    public static final double WHEEL_DIAMETER_MM = 104.0;
    public static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES;
    public static final double DEFAULT_TICKS_PER_WHEEL_REVOLUTION = 537.7;
    public static final double DEFAULT_TRACK_WIDTH_INCHES = 15.0;
    public static final double DEFAULT_WHEEL_BASE_INCHES = 15.0;
}

