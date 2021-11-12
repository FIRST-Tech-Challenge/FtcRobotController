package org.firstinspires.ftc.teamcode.utils;

/**
 * This class has all constant definitions.
 */
public class FTCConstants {
    /**
     * Ticks every revolution
     */
    public static final double COUNTS_PER_MOTOR_REV = 288;

    /**
     * Drive gear (Ratios, we don't need to worry about this as we're using a hex core)
     */
    public static final double DRIVE_GEAR_REDUCTION = 1.0;

    /**
     * For figuring circumference
     */
    public static final double WHEEL_DIAMETER_INCHES = 3.54331;

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
    public static final String LEFT_MOTOR_NAME = "motorLeft";

    /**
     * The identifier of the right motor from the robot.
     */
    public static final String RIGHT_MOTOR_NAME = "motorRight";

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
