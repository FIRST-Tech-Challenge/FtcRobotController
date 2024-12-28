package org.firstinspires.ftc.teamcode.Hardware.Constants;

public class DepositConstants {

    // Claw Positions
    public static final double
            clawClosedPos = 0.035,
            clawOpenPos = 0.46;

    public static final double
            clawEncClosedPos = 307.00,
            clawEncOpenPos = 189.00,
            clawEncPosTolerance = 3.00;

    // Arm Positions
    public static final double
            armRightTransferPos = 0.822,
            armRightSpecIntakePos = 0.1,
            armRightSpecDepositPos = 0.81,
            armRightSampleDepositPos = 0.3;

    // Arm Enc Positions
    public static final double
            armRightEncTransferPos = 76.00,
            armRightEncSpecIntakePos = 308.20,
            armRightEncSpecDepositPos = 79.00,
            armRightEncSampleDepositPos = 243.70,
            armRightEncSlideDownSafePos = 165.00,
            armRightPositionTolerance = 10.00;

    // Deposit Slide PID Constants
    public static final double
            sp = 0.032,
            si = 0.05,
            sd = 0.0005,
            sf = 0.00;
    //Zeros for testing
//            sp = 0.0,
//            si = 0.0,
//            sd = 0.0,
//            sf = 0.0;

    // Deposit Slides Positions in cm
    public static final double
            slideTransferPos = 8.50,
            slidePreTransferPos = 21.50,
            slideSpecIntakePos = 0.00,
            slideSpecDepositReadyPos = 48.00,
            slideSpecClippedPos = 65.50,
            slideSampleDepositPos = 72,
            slideMaxExtension = 72,
            slidePositionTolerance = 1.00;



}
