package org.firstinspires.ftc.team6220_PowerPlay;

public class Constants {
    public static final double INCHES_PER_METER = 100 / 2.54;

    public static final double CORRECTION_CONSTANT = 1 / 45.0; // constant for converting angle error to motor speed

    public static final int SLIDE_TOP = 3900;
    public static final int SLIDE_HIGH = 3800;
    public static final int SLIDE_MEDIUM = 2750;
    public static final int SLIDE_LOW = 1650;
    public static final int SLIDE_STOW = 300;
    public static final int SLIDE_BOTTOM = 0;

    public static final double DRIVE_SPEED_MULTIPLIER = 0.6;
    public static final int DRIVE_DEADZONE_DEGREES = 45;

    public static final double GRABBER_INITIALIZE_POSITION = 0.7;
    public static final double GRABBER_OPEN_POSITION = 0.25;
    public static final double GRABBER_CLOSE_POSITION = 0.0;

    public static final double WHEEL_CIRCUMFERENCE = 96 / 25.4 * Math.PI;
    public static final double DRIVE_MOTOR_TICKS_TO_INCHES = WHEEL_CIRCUMFERENCE * Math.sqrt(2) / 537.6;
}
