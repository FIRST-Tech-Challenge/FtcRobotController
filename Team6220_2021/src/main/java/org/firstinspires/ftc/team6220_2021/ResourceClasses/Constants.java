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

    // Motor Constants
    public static final int BELT_RESET = 1250;
    public static final int ARM_COLLECTING_LEVEL = 100;
    // todo - find values
    public static final int ARM_ALLIANCE_HUB_1ST_LEVEL = 300;
    public static final int ARM_SHARED_HUB_LEVEL = 500;
    public static final int ARM_ALLIANCE_HUB_3RD_LEVEL = 650;
    public static final int ARM_BACKWARDS_ALLIANCE_HUB_3RD_LEVEL = 1800;
    public static final int ARM_CAPPING_LEVEL = 800;

    // Servo Values
    public static final double CLOSED_GRABBER_POSITION = 0.3;
    public static final double OPEN_GRABBER_POSITION = 0.5;
    public static final double SERVO_ARM_SHARED_HUB_POSITION = 0.0;
    public static final double SERVO_ARM_RESET_POSITION = 0.45;

    // TensorFlow Autonomous
    public static final String VUFORIA_KEY = "AXDNhID/////AAABmTzx9+zSP0cgsSvEBLeS2Y9I1y9lY1nEbJ0" +
            "/cUmIw6GzDXvrllKLQizl4X4T6iAxXFMJXR5zS8fcXuy6uS6lzlZJOBRnDXn3FusCpuunkIRPgIVyq+peMid" +
            "0PN1gwSloq8A+nrV6W1LU10WzZ/Pez2F0to+5aV0bOBB+VhZIdN5ABnoSMPa6JxtR6QaCI3dg++wpGw+/X3R" +
            "wDJhllOoGVmsLE9DTEuBBAI+MtRIpFNrSR7mcv3TEHMf8YIc+qxED8YE7Az3PGK1xy/NzLqNtFdnNVFhp023" +
            "8Kaaqnu3DABLRXRjSJ1QRSHmE8mIur5Dk3OcqMv3fwTNt5CnhC2J/D5biVGixUQ+dveylNEVNmp0k";
    public static final String[] TENSORFLOW_LABELS = {"TSE, Cube, Sphere, Duck"};
    public static final String TENSORFLOW_MODEL_ASSET = "model_20220108_181014.tflite";
}