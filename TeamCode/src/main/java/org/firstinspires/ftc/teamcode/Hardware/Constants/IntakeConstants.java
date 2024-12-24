package org.firstinspires.ftc.teamcode.Hardware.Constants;

public class IntakeConstants {

    // Intake Slide PID Constants
    public static final double
            sp = 0.015,
            si = 0.00,
            sd = 0.00;

    // Intake Slide Positions in CM
    public static final double
            stowedPosition = 0.00,
            readyPosition = 30.00,
            minIntakePosition = 10.00,
            maxExtensionPosition = 70.00,
            intakeSlidePositionTolerance = 0.50;

    // Max Intake Slide Manual Feed Rate in cm/s
    public static final double
            maxFeedRate = 200.00;


    // Bucket Servo Positions
    public static final double
            bucketUpPosition = 0.175,
            bucketDownPosition = 0.395;

    // Bucket Encoder Positions & Tolerance
    public static final double
            bucketEncUpPosition = 283.00,
            bucketEncDownPosition = 214.00,
            bucketEncPositionTolerance = 3.00;

    // Gate Positions
    public static final double
            gateOpenPosition = 0.00,
            gateBlockedPosition = 0.27;

    // Intake Motor Powers
    public static final double
            intakingPower = 1.00,
            stallingPower = 0.30;

    // Detection Distance in mm
    public static final double
            detectionDistance = 30.00;

    // SampleDetector max values
    public static final double
            maxR = 620.0,
            maxG = 1160.00,
            maxB = 1075.00,
            maxA = 1000.00;

}