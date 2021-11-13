package org.firstinspires.ftc.teamcode.Enhancement.Config;

import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Drive.Drive;

/**
 * Drive Config has the static config values for the drive class in drive.java
 *
 * <p>This improves the readability of drive.java and is only for static variables, not for non-static variables or methods.</p>
 *
 * @see Drive
 */
public class DriveConfig extends Config {

    public static final double ODOMETRY_mm_PER_COUNT = 38.85 * 3.14159265 / 8192.0;
    public static final double ODOMETRY_RADIUS_X = 201.0;
    public static final double ODOMETRY_RADIUS_Y = 178.0;

    // NEW Chassis
    public static final double MOTOR_TICK_PER_REV_YELLOJACKET312 = 537.6;
    public static final double GOBUILDA_MECANUM_DIAMETER_MM = 96.0;
    public static final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4;     // For figuring circumference
    public static final double WHEEL_DIAMETER_MM = 100.0;
    public static final double COUNTS_CORRECTION_X = 1.167;
    public static final double COUNTS_CORRECTION_Y = 0.9918;
    public static final double COUNTS_PER_DEGREE = 10.833 * 0.99;     // 975 ticks per 90 degrees
    public static final double DRIVE_SPEED = 0.40;
    public static final double DRIVE_SPEED_X = 0.35;
    public static final double DRIVE_SPEED_Y = 0.40;
    public static final double TURN_SPEED = 0.40;
    public static final double ROBOT_INIT_POS_X = 15.0;
    public static final double ROBOT_INIT_POS_Y = 15.0;
    public static final double ROBOT_INIT_ANGLE = 45.0;
    public static final float mmPerInch = 25.4f;
    // DO WITH ENCODERS
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double TICKS_PER_MOTOR_REV_20 = 537.6;    // AM Orbital 20 motor
    public static final double COUNTS_PER_INCH = (TICKS_PER_MOTOR_REV_20 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double COUNTS_PER_MM = (MOTOR_TICK_PER_REV_YELLOJACKET312 * DRIVE_GEAR_REDUCTION) / (GOBUILDA_MECANUM_DIAMETER_MM * Math.PI);
    public static final double RPM_MAX_NEVERREST_20 = 340;
    public static final double ANGULAR_V_MAX_NEVERREST_20 = (TICKS_PER_MOTOR_REV_20 * RPM_MAX_NEVERREST_20) / 60.0;
    public static int encoderOffsetFL = 0;
    public static int encoderOffsetFR = 0;
    public static int encoderOffsetRL = 0;
    public static int encoderOffsetRR = 0;
    public static int odometryCountOffsetL = 0;
    public static int odometryCountOffsetR = 0;
    public static int odometryCountOffsetB = 0;
    public static int odometryCountL = 0;
    public static int odometryCountR = 0;
    public static int odometryCountB = 0;
    public static boolean driveFullPower = false;
    public static double motorKp = 0.015;
    public static double motorKi = 0.02;
    public static double motorKd = 0.0003;
    public static double motorRampTime = 0.3;

    @Override
    public Object get(String key) {
        return null;
    }

    @Override
    public void set(String key, Object value) {

    }
}
