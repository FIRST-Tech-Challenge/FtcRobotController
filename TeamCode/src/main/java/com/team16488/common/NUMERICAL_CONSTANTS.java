package com.team16488.common;

/**
 * Here we store all the static variables that will not change throughout our robot
 * like ticks per rotation or port numbers for custom drivers
 * this way we have everything in one place
 *
 */
public class NUMERICAL_CONSTANTS {
    //ex: public static final int TICKS_PER_INCH = 1240
    public static final double     TICKS_PER_MOTOR_REV     = 1440 ;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     TICKS_PER_INCH          = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);

}
