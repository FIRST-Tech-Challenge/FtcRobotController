package org.firstinspires.ftc.team6220_PowerPlay;

public class Constants {
    public static final double INCHES_PER_METER = 100 / 2.54;

    public static final double CORRECTION_CONSTANT = 1 / 45.0; // constant for converting angle error to motor speed

    public static final int SLIDE_TOP = 3900;
    public static final int SLIDE_HIGH = 3800;
    public static final int SLIDE_MEDIUM = 2750;
    public static final int SLIDE_LOW = 1650;
    public static final int SLIDE_STOW = 600;
    public static final int SLIDE_STACK_FOUR = 320;
    public static final int SLIDE_STACK_THREE = 240;
    public static final int SLIDE_STACK_TWO = 160;
    public static final int SLIDE_STACK_ONE = 80;
    public static final int SLIDE_BOTTOM = 0;

    public static final double GRABBER_INITIALIZE_POSITION = 1.0;
    public static final double GRABBER_OPEN_POSITION = 0.57;
    public static final double GRABBER_CLOSE_POSITION = 0.0;

    public static final double WHEEL_CIRCUMFERENCE = 96 / 25.4 * Math.PI;
    public static final double DRIVE_MOTOR_TICKS_TO_INCHES = WHEEL_CIRCUMFERENCE * Math.sqrt(2) / 537.6;

    public static final String VUFORIA_KEY = "AXDNhID/////AAABmTzx9+zSP0cgsSvEBLeS2Y9I1y9lY1nEbJ0" +
            "/cUmIw6GzDXvrllKLQizl4X4T6iAxXFMJXR5zS8fcXuy6uS6lzlZJOBRnDXn3FusCpuunkIRPgIVyq+peMid" +
            "0PN1gwSloq8A+nrV6W1LU10WzZ/Pez2F0to+5aV0bOBB+VhZIdN5ABnoSMPa6JxtR6QaCI3dg++wpGw+/X3R" +
            "wDJhllOoGVmsLE9DTEuBBAI+MtRIpFNrSR7mcv3TEHMf8YIc+qxED8YE7Az3PGK1xy/NzLqNtFdnNVFhp023" +
            "8Kaaqnu3DABLRXRjSJ1QRSHmE8mIur5Dk3OcqMv3fwTNt5CnhC2J/D5biVGixUQ+dveylNEVNmp0k";

    public static final String[] TENSORFLOW_LABELS = {"BlueCone", "RedCone", "JunctionBottom", "JunctionTop"};
    //public static final String TENSORFLOW_MODEL_ASSET = ";

    // TensorFlow Object Detection
    public static final int ROBOT_CAMERA_CENTER_Y = 540;
    public static final int ROBOT_CAMERA_CENTER_X = 960;
    public static final int GRABBER_CAMERA_CENTER_Y = 240;
    public static final int GRABBER_CAMERA_CENTER_X = 320;
}