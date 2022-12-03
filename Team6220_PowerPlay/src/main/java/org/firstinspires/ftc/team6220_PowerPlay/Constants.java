package org.firstinspires.ftc.team6220_PowerPlay;

public class Constants {
    public static final double INCHES_PER_METER = 100 / 2.54;

    public static final int TURNTABLE_DEFAULT_POSITION = -150;

    public static final double CORRECTION_CONSTANT = 1 / 45.0; // constant for converting angle error to motor speed

    public static final int SLIDE_TOP = 9600;
    public static final int SLIDE_HIGH = 7900;
    public static final int SLIDE_MEDIUM = 5500;
    public static final int SLIDE_LOW = 3100;
    public static final int SLIDE_BOTTOM = 0;

    public static final double SLIDE_P_CONSTANT = 0.001;
    public static final double SLIDE_DEADZONE = 0.1;

    public static final double DRIVE_SPEED_MULTIPLIER = 0.4;
    public static final int DRIVE_DEADZONE_DEGREES = 45;
    public static final double DRIVE_MOVE_CURVE_FAC = 0.4;
    public static final double DRIVE_TURN_CURVE_FAC = 0.4;
    public static final double DRIVE_STICK_DEADZONE = 0.025;

    public static final double TURNTABLE_DEADZONE = 0.1;

    public static final double GRABBER_OPEN_POSITION = 0.7;
    public static final double GRABBER_CLOSE_POSITION = 0.0;

    public static final double WHEEL_CIRCUMFERENCE = 96 / 25.4 * Math.PI;
    public static final double DRIVE_MOTOR_TICKS_TO_INCHES = WHEEL_CIRCUMFERENCE * 2 / 537.6;
}
