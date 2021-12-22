package org.firstinspires.ftc.team6220_2021.ResourceClasses;

/**
 Used to store important constants for easy access in other classes.
 */

public class Constants {
    // Standard Conversions
    public static final double AM_40_TICKS_PER_ROTATION = 1120;
    public static final double AM_20_TICKS_PER_ROTATION = 537.6;
    public static final double AM_37_TICKS_PER_ROTATION = 103.6;

    // Robot Specs
    public static final double WHEEL_DIAMETER_IN = 4.0;  // 4 inch diameter wheel
    public static final double SPROCKET_RATIO = 1.0;  // Driven to driving sprocket teeth
    public static final double IN_PER_AM_TICK = (Math.PI * WHEEL_DIAMETER_IN) /
            (AM_20_TICKS_PER_ROTATION * SPROCKET_RATIO);  // Num inches per tick of drive motors

    // Movement Control Constants
    // todo - adjust for new chassis
    public static final double MIN_DRIVE_PWR = 0.1;
    public static final double MIN_TURN_PWR = 0.25;
    // Constants for adjusting powers that are proportional to angle and position differences
    public static final double DRIVE_POWER_FACTOR = 0.03;
    public static final double TURNING_POWER_FACTOR = 0.005;

    // todo Implement I and D terms; may need to adjust for new chassis
    // PID Loop Constants
    public static final double ROTATION_P = TURNING_POWER_FACTOR;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final double TRANSLATION_P = DRIVE_POWER_FACTOR;
    public static final double TRANSLATION_I = 0.0;
    public static final double TRANSLATION_D = 0.0;

    // Joystick and Trigger Dead Zones
    public static final double MINIMUM_JOYSTICK_POWER = 0.1;
    public static final double MINIMUM_TRIGGER_VALUE = 0.1;

    // Servo Values
    // servo constants go here
}