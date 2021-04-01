package org.firstinspires.ftc.team6220_2020;

/**
 Used to store important constants for easy access in other classes.
 */

public class Constants
{
    // Standard conversions------------------------------------------
    // This is for an AndyMark 40; 20s are 560 ticks / rot and 60s are 1680 ticks / rot
    public static final int AM_40_TICKS_PER_ROTATION = 1120;
    public static final int US_DIGITAL_TICKS_PER_ROTATION = 1440;   // For odometry wheels
    public static final double MM_PER_INCH = 25.4;
    public static final double IN_FIELD_SIZE = 140.94;  // Not 144" due to interlocking pieces being cut off on field edge.
    public static final double AM_37_TICKS_PER_ROTATION = 103.6;
    //---------------------------------------------------------------


    // Tolerances----------------------------------------------------
    public static final double ANGLE_TOLERANCE_DEG =  1.0;
    public static final double POSITION_TOLERANCE_IN = 0.5;
    public static final double OPENCV_TOLERANCE_PIX = 0.5;
    public static final double LIFT_MOTOR_TOLERANCE_ENC_TICKS = 30; // Tolerance for automatic lift movements
    //---------------------------------------------------------------


    // Robot specs---------------------------------------------------
    public static final double WHEEL_DIAMETER_IN = 4.0;         // 4 inch diameter wheel
    public static final double SPROCKET_RATIO = 16.0 / 16.0;    // Driven to driving sprocket teeth
    public static final double IN_PER_ANDYMARK_TICK = (Math.PI * WHEEL_DIAMETER_IN) / (0.5 * AM_40_TICKS_PER_ROTATION * SPROCKET_RATIO);    // Num inches per tick of drive motors
    //---------------------------------------------------------------


    // Drive mode constants------------------------------------------
    public static final double SLOW_MODE_T_FACTOR = 0.3;        // Slow mode multipliers
    public static final double SLOW_MODE_R_FACTOR = 0.3;
    public static final double T_FACTOR = 1.0;                  // Normal multipliers
    public static final double R_FACTOR = 1.0;
    public static final double MAX_NAV_ROT_POWER = 0.3;         // Max rotation power when navigating
    public static final double AUTO_SEARCH_TURN_POWER = 0.2;    // Power when looking for image targets
    //---------------------------------------------------------------


    // Movement control constants------------------------------------
    public static final double MINIMUM_DRIVE_POWER = 0.08;
    public static final double MAX_DRIVE_POWER = 1.0;
    public static final double MINIMUM_TURNING_POWER = 0.02;    // todo Too small?
    // Constants for adjusting powers that are proportional to angle and position differences
    public static final double TURNING_POWER_FACTOR = 0.01;
    public static final double DRIVE_POWER_FACTOR = 0.04;
    //---------------------------------------------------------------


    // todo Implement I and D terms; may need to adjust for new chassis
    // PID loop constants--------------------------------------------
    public static final double ROTATION_P = TURNING_POWER_FACTOR;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final double TRANSLATION_P = DRIVE_POWER_FACTOR;
    public static final double TRANSLATION_I = 0;
    public static final double TRANSLATION_D = 0;
    //----------------------------------------------------------------


    // Vuforia and OpenCV constants----------------------------------
    public static final int IMAGE_WIDTH = 1280;
    public static final int IMAGE_HEIGHT = 720;
    public static final double WEBCAM_1_OFFSET = 0;    // Need to account for fact that webcam may face different direction from drivetrain.
    public static final double WEBCAM_2_OFFSET = 0;    // * 2nd webcam not currently in use.
    //---------------------------------------------------------------

    
    // Joystick and trigger dead zones to ensure that input isn't used when controller registers small perturbations.
    public static final double MINIMUM_JOYSTICK_POWER = 0.05;
    public static final double MINIMUM_TRIGGER_VALUE = 0.1;
    //---------------------------------------------------------------

    // Helpful Constants
    public static  final double MILLIS_TO_MIN = 60000;
    //--------------------------------------------------------------
}
