package org.firstinspires.ftc.team6220_PowerPlay;

import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class Constants {
    public static final double INCHES_PER_METER = 100 / 2.54;

    public static final double HEADING_CORRECTION_KP_TELEOP = 0.03;
    public static final double HEADING_CORRECTION_KP_AUTONOMOUS = 0.001;

    public static final double TURNING_KP = 0.008;

    public static final int UNIT_CIRCLE_OFFSET_DEGREES = 90;

    public static final double DRIVE_CURVE_FACTOR = 0.7;

    public static final double MINIMUM_TURN_POWER = 0.05;
    public static final double MINIMUM_DRIVE_POWER = 0.05;

    public static final double MAXIMUM_TURN_POWER_AUTONOMOUS = 0.3;
    public static final double MAXIMUM_DRIVE_POWER_AUTONOMOUS = 0.3;

    public static final double MAXIMUM_TURN_POWER_TELEOP = 0.5;
    public static final double MAXIMUM_DRIVE_POWER_TELEOP = 0.5;

    public static final double SLIDE_FEEDFORWARD = 0.05;
    public static final double SLIDE_MOTOR_KP = 0.01;

    public static final int MIN_SLIDE_ERROR_FULL_POWER = -200;

    public static final int ROBOT_HEADING_TOLERANCE_DEGREES = 1;
    public static final int ROBOT_SLIDE_TOLERANCE_TICKS = 20;

    public static final int SLIDE_TOP = 3900;
    public static final int SLIDE_HIGH = 3800;
    public static final int SLIDE_MEDIUM = 2750;
    public static final int SLIDE_LOW = 1650;
    public static final int SLIDE_STOW = 500;
    public static final int SLIDE_STACK_FOUR = 650;
    public static final int SLIDE_STACK_THREE = 490;
    public static final int SLIDE_STACK_TWO = 330;
    public static final int SLIDE_STACK_ONE = 170;
    public static final int SLIDE_BOTTOM = 0;

    public static final double GRABBER_INITIALIZE_POSITION = 0.17;
    public static final double GRABBER_OPEN_POSITION = 0.46;
    public static final double GRABBER_CLOSE_POSITION = 1.0;

    public static final double WHEEL_CIRCUMFERENCE = 96 / 25.4 * Math.PI;
    public static final double DRIVE_MOTOR_TICKS_TO_INCHES = WHEEL_CIRCUMFERENCE * Math.sqrt(2) / 537.6;
    public static final int DRIVE_MOTOR_INCHES_TO_TICKS = 379;

    public static final int CAMERA_X = 800;
    public static final int CAMERA_Y = 600;

    public static final int CAMERA_CENTER_X = CAMERA_X / 2;
    public static final int CAMERA_CENTER_Y = CAMERA_Y / 2;

    public static final int CONE_WIDTH = 350;
    public static final double CONE_STACK_WIDTH_KP = -0.00005;
    public static final double CONE_STACK_CENTERING_KP = -0.0005;
    public static final double CONE_STACK_CENTERING_PROPORTIONAL_KP = -0.0003;
    public static final double CONE_STACK_CENTERING_MAX_SPEED = 0.25;

    public static final int JUNCTION_TOP_TOLERANCE = 100;
    public static final double JUNCTION_TOP_CENTERING_KP = 0.0004;

    public static final Scalar LOWER_RED = new Scalar(20, 0, 0);
    public static final Scalar UPPER_RED = new Scalar(160, 255, 252);

    public static final Scalar LOWER_BLUE = new Scalar(100, 125, 75);
    public static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);

    public static final Scalar LOWER_YELLOW = new Scalar(15, 75, 100);
    public static final Scalar UPPER_YELLOW = new Scalar(35, 255, 255);

    public static final Scalar LOWER_BLACK = new Scalar(0, 0, 0);
    public static final Scalar UPPER_BLACK = new Scalar(180, 255, 40);

    public static final Size BLUR_SIZE = new Size(5, 5);

    public static final int DISTANCE_FROM_CENTER_JUNCTION_TOP = 200;

    public static final int CONTOUR_MINIMUM_SIZE = 200;

    public static final double TURNING_AUTHORITY_CONSTANT = 250.0;
    public static final double STRAFING_AUTHORITY_CONSTANT = 200.0;

    public static final double AUTHORITY_SCALER = 1.09;

    public static final double DRIVE_AUTHORITY_SCALER = 1.35;

    public static final int AUTONOMOUS_STACK_BASE_OFFSET = 0;

    public static final int AUTONOMOUS_STACK_PER_CONE_OFFSET = 160;

    public static final int JUNCTION_TOP_MAX_SIZE = 40000;

    public static final int[] STACK_HEIGHTS = new int[] {0, 117, 238, 383, 537};
}
