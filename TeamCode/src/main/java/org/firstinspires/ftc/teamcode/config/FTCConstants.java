package org.firstinspires.ftc.teamcode.config;


/**
 * This class has all constant definitions.
 * @author aryansinha
 * and also less significantly
 * @author karthikperi
 */
public class FTCConstants {
    /**
     * Ticks every revolution
     */
    public static final double COUNTS_PER_MOTOR_GOBILDA = 537.7;
    /**
     * Drive gear (Ratios, we don't need to worry about this as we're using a hex core)
     */
    public static final double DRIVE_GEAR_REDUCTION = 19.1986;
    /**
     * For figuring circumference
     */
    public static final double WHEEL_DIAMETER_INCHES = 3.77953;
    /**
     * Counts per inch.
     */
    public static final double COUNTS_PER_INCH = (WHEEL_DIAMETER_INCHES * 3.1415) / (COUNTS_PER_MOTOR_GOBILDA);
}