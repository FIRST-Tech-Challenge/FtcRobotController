package org.firstinspires.ftc.teamcode.Configs.utils;

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
    public static final double COUNTS_PER_MOTOR_REV = 28;

    /**
     * Drive gear (Ratios, we don't need to worry about this as we're using a hex core)
     */
    public static final double DRIVE_GEAR_REDUCTION = 10.5556;

    /**
     * For figuring circumference
     */
    public static final double WHEEL_DIAMETER_INCHES = 2.8346457;

    /**
     * Counts per inch.
     */
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    /**
     * The driving speed that we want. Change here if we want to increase this.
     */
    public static final double DRIVE_SPEED = 0.4;

    /**
     * The turn speed that we want. Change here if we want to increase or decrease this.
     */
    public static final double TURN_SPEED = 0.2;

    // Following are the definitions of our hardware components for references from the actual robot.
    // If you add , remove or rename a component, this is the place to reflect the new change.

    /**
     * The identifier of the left motor from the robot.
     */
    public static final String LEFT_MOTOR_NAME = "leftFrontMotor";

    /**
     * The identifier of the right motor from the robot.
     */
    public static final String RIGHT_MOTOR_NAME = "rightFrontMotor";

    public static final String BLEFT_MOTOR_NAME = "leftBackMotor";

    /**
     * The identifier of the right motor from the robot.
     */
    public static final String BRIGHT_MOTOR_NAME = "rightBackMotor";

    /**
     * The identifier of the claw.
     */
    public static final String CLAW_NAME = "claw";

    /**
     * The identifier of the claw servo.
     */
    public static final String CLAW_SERVO = "clawServo";

    /**
     * The identifier of the carousel servo.
     */
    public static final String CAROUSEL_SERVO = "carouselServo";
}
