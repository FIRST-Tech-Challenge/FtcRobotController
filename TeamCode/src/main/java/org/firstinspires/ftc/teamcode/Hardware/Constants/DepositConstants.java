package org.firstinspires.ftc.teamcode.Hardware.Constants;

public class DepositConstants {

    // Claw Positions
    public static final double
            clawClosedPos = 0.035,
            clawOpenPos = 0.46;

    public static final double
            clawEncClosedPos = 311.00,
            clawEncOpenPos = 192.50,
            clawEncPosTolerance = 4.00;

    // Arm Positions
    public static final double
            armRightTransferPos = 0.932,
            armRightSpecIntakePos = 0.04,
            armRightSpecDepositPos = 0.89,
            armRightSampleDepositPos = 0.32;

    // Arm Enc Positions
    public static final double
            armRightEncTransferPos = 42.00,
            armRightEncSpecIntakePos = 330.50,
            armRightEncSpecDepositPos = 53.60,
            armRightEncSampleDepositPos = 240.70,
            armRightEncSlideDownSafePos = 165.00,
            armRightPositionTolerance = 10.00;

    // Deposit Slide PID Constants
    public static final double
            sp = 0.02,
            si = 0.05,
            sd = 0.00035,
            sf = 0.13;
    //Zeros for testing
//            sp = 0.0,
//            si = 0.0,
//            sd = 0.0,
//            sf = 0.0;

    // Deposit Slides Positions in cm
    public static final double
            slideTransferPos = 4.00,
            slidePreTransferPos = 20.00,
            slideSpecIntakePos = 0.00,
            slideSpecDepositReadyPos = 44.90,
            slideSpecClippedPos = 63.00,
            slideSampleDepositPos = 72,
            slideMaxExtension = 72,
            slidePositionTolerance = 1.00;



}
