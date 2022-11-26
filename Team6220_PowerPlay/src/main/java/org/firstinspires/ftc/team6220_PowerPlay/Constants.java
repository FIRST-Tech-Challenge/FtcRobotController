package org.firstinspires.ftc.team6220_PowerPlay;

public class Constants {
    public static final double INCHES_PER_METER = 100 / 2.54;

    public static final int TURNTABLE_DEFAULT_POSITION = -150;

    public static final double CORRECTION_CONSTANT = 1 / 45.0; // constant for converting angle error to motor speed

    public static final int VERTICAL_SLIDE_TOP = 9600;
    public static final int VERTICAL_SLIDE_HIGH_JUNCTION = 7900;
    public static final int VERTICAL_SLIDE_MEDIUM_JUNCTION = 5500;
    public static final int VERTICAL_SLIDE_LOW_JUNCTION = 3100;
    public static final int VERTICAL_SLIDE_BOTTOM = 0;

    public static final double VERTICAL_SLIDE_P_CONSTANT = 0.001;
    public static final double VERTICAL_SLIDE_DEADZONE = 0.1;

    public static final double DRIVE_SPEED_MULTIPLIER = 1.0;
    public static final int DRIVE_DEADZONE_DEGREES = 20;

    public static final double TURNTABLE_DEADZONE = 0.1;

    public static final double GRABBER_OPEN_POSITION = 0.7;
    public static final double GRABBER_CLOSE_POSITION = 0.0;
}
