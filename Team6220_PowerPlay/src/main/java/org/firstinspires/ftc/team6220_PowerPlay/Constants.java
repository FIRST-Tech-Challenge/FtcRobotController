package org.firstinspires.ftc.team6220_PowerPlay;

import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class Constants {
    public static final double INCHES_PER_METER = 100 / 2.54;

    public static final double HEADING_CORRECTION_KP = 0.03;

    public static final double DRIVE_CURVE_FACTOR = 0.4;
    public static final double MINIMUM_TURNING_POWER = 0.05;
    public static final double MAXIMUM_TURNING_POWER = 0.25;
    public static final double MAXIMUM_DRIVE_POWER = 0.5;
    public static final double SLIDE_FEEDFORWARD = 0.05;
    public static final double SLIDE_MOTOR_KP = 0.01;

    public static final int ROBOT_HEADING_TOLERANCE_DEGREES = 1;
    public static final int ROBOT_SLIDE_TOLERANCE_TICKS = 20;

    public static final int SLIDE_TOP = 3900;
    public static final int SLIDE_HIGH = 3800;
    public static final int SLIDE_MEDIUM = 2750;
    public static final int SLIDE_LOW = 1650;
    public static final int SLIDE_STOW = 500;
    public static final int SLIDE_STACK_FOUR = 600;
    public static final int SLIDE_STACK_THREE = 450;
    public static final int SLIDE_STACK_TWO = 300;
    public static final int SLIDE_STACK_ONE = 180;
    public static final int SLIDE_BOTTOM = 0;

    public static final double GRABBER_INITIALIZE_POSITION = 1.0;
    public static final double GRABBER_OPEN_POSITION = 0.6;
    public static final double GRABBER_CLOSE_POSITION = 0.0;

    public static final double WHEEL_CIRCUMFERENCE = 96 / 25.4 * Math.PI;
    public static final double DRIVE_MOTOR_TICKS_TO_INCHES = WHEEL_CIRCUMFERENCE * Math.sqrt(2) / 537.6;

    public static final int CAMERA_X = 800;
    public static final int CAMERA_Y = 600;

    public static final int CAMERA_CENTER_X = CAMERA_X / 2;
    public static final int CAMERA_CENTER_Y = CAMERA_Y / 2;

    public static final int CONE_WIDTH = 350;
    public static final double CONE_CENTERING_KP = 0.035;

    public static final int JUNCTION_TOP_TOLERANCE = 25;
    public static final double JUNCTION_TOP_CENTERING_KP = 0.1;

    public static final Scalar LOWER_RED = new Scalar(170, 125, 50);
    public static final Scalar UPPER_RED = new Scalar(180, 255, 255);

    public static final Scalar LOWER_BLUE = new Scalar(100, 125, 75);
    public static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);

    public static final Scalar LOWER_YELLOW = new Scalar(15, 75, 100);
    public static final Scalar UPPER_YELLOW = new Scalar(35, 255, 255);

    public static final Scalar LOWER_BLACK = new Scalar(0, 0, 0);
    public static final Scalar UPPER_BLACK = new Scalar(180, 255, 40);

    public static final Size BLUR_SIZE = new Size(5, 5);
}
