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
            maxExtensionPosition = 63.50,
            intakeSlidePositionTolerance = 1.8;

    // Max Intake Slide Manual Feed Rate in cm/s
    public static final double
            maxFeedRate = 200.00;

    public static final double
            intakeSlideZeroPower = -1.0,
            intakeSlideZeroStallPower = -0.1;


    // Bucket Servo Positions
    public static final double
            bucketUpPosition = 0.425,
            bucketDownPosition = 0.635;

    // Bucket Encoder Positions & Tolerance
    public static final double
            bucketEncUpPosition = 206.50,
            bucketEncDownPosition = 139.50,
            bucketEncDownPartialPosition = (bucketEncUpPosition + bucketEncDownPosition) / 2.0,
            bucketEncPositionTolerance = 5.00;

    // Gate Positions
    public static final double
            gateOpenPosition = 0.465,
            gateBlockedPosition = 0.17;

    // Intake Motor Powers
    public static final double
            intakingPower = 1.00,
            stallingPower = 0.450;

    // Detection Distance in mm
    public static final double
            detectionDistance = 25.00;

    // SampleDetector max values
    public static final double
            maxR = 2380.0,
            maxG = 4500.00,
            maxB = 3750.00,
            maxA = 3550.00;

}