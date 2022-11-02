package org.firstinspires.ftc.team6220_PowerPlay;

public class Constants {
    public static final int DEADZONE_ANGLE_DEGREES = 10;

    public static final double CORRECTION_CONSTANT = 1 / 45.0; // constant for converting angle error to motor speed

    public static final double DRIVE_SPEED_MULTIPLIER = 0.5;

    public static final double INCHES_PER_METER = 39.37;

    public static final double WHEEL_DIAMETER_INCHES = 96 / 2.54;

    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;

    public static final double AM20_TICKS_PER_REVOLUTION = 537.6;

    public static final double CHASSIS_INCHES_PER_TICK = WHEEL_CIRCUMFERENCE_INCHES / AM20_TICKS_PER_REVOLUTION / Math.sqrt(2);

    public static final double VERTICAL_SLIDE_SPEED_MULTIPLIER = 0.5;

    public static final double VERTICAL_SLIDE_DEADZONE = 0.1;

    public static final double GRABBER_OPEN_POSITION = 0.33;

    public static final double GRABBER_CLOSE_POSITION = 0.11;
}
