package org.firstinspires.ftc.teamcode.params;

public class DriveParams {
    /*********************** HARDWARE CONSTANTS **************************************************/
    // 12.6 inches circumference of a wheel
    public static final double CIRCUMFERENCE = 12.6;
    // 1120 ticks in a revolution
    public static final double TICKS_PER_REV = 560;
    // Is the IMU unit vertical?
    public static final boolean IMU_VERT     = false;

    /*********************** TUNABLE GAME CONSTANTS **************************************************/
    public static final double SLOW_DRIVE_COEFF = 0.4;

    /*********************** TUNABLE DRIVE CONSTANTS **************************************************/
    // Gives the point at which to switch to less than full power in a turn
    public static final double FULL_POWER_UNTIL  = 160;
    // Minimum speed to complete the turn
    public static final double MIN_SPEED         = 0.2;
    public static final double STRAFE_COEFF      = 0.09;
    // Larger P_TURN is more responsive, but also less stable
    // 0.012
    public static final double P_TURN_COEFF      = 0.015;
    // Tightest threshold as we can make it with an integer gyro
    public static final double HEADING_THRESHOLD = 5 ;
    // Acceptable distance in ticks to stop motors at
    public static final int TICK_TOLERANCE       = 15;

}