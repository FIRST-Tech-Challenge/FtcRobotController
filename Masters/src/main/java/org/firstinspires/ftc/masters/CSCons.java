package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class CSCons {

    public static double skewampus = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double pixelDetectThreshold = 230;
    public static double clawOpen = 0.4;
    public static double clawClosed = 0.82;
    public static double clawTransfer = 0.72;
    public static double clawAngleGroundToThree = .87;
    public static double clawAngleFourToFive = 1;
    public static double clawAngleTransition = .2;
    public static double clawAngleTransfer = .02;
    public static double clawArmGround = .75;
    public static double clawArm2 = .795;
    public static double clawArm3 = .755;
    public static double clawArm4 = .73;
    public static double clawArm5 = .695;
    public static double clawArmTransition = .2;
    public static double clawArmTransfer = .2;

    //public static double[]outtakeAngle={0.234, 0.9}; //folder, transfer
    public static double outtakeAngleFolder=.05;
    public static double outtakeAngleTransfer=67;
    //public static double[] doubleServoBack= {0.7, 0.15}; //drop, transfer
    public static double outtakeMovementBackDrop = .54;
    public static double outtakeMovementBackTransfer = .064;
    //public static double[] rightSideBack = {0.955, 0.65};
    public static double openHook = 0.8;
    public static double closeHook = 1;

    public static double droneFlat = 0.3;
    public static double droneShooting = 0.1;

    public static double closeClawDistance = 3; //in cm

    public static long transferToBottomIntake = 200; //time in ms
    public static long transferToScoreOuttake = 200;
    public static long scoreToTransferOuttake = 200;
    public static long closingHook = 100;

    public enum OuttakePosition{
        BOTTOM (30),
        LOW(500),
        MID(1000),
        HIGH(2600);

        private int target;

        private OuttakePosition(int target){
            this.target = target;
        }

        public int getTarget(){
            return  target;
        }
    }

    public enum IntakeState {
        Transition, Transfer, Intake, MoveToIntake;
    }

    public enum OuttakeState{
        ClosingHook, MoveToTransfer, ReadyToTransfer, MoveToDrop, ReadyToDrop, Align, BackUp
    }

    public enum DriveMode {
        NORMAL,
        PIXEL_SCORE,
        END_GAME
    }

    public enum ClawPosition{
        OPEN, CLOSED, TRANSFER
    }

    public enum  HookPosition{
        OPEN, CLOSED
    }

/**
 *
 *  ╚ ╔ ╩ ╦ ╠ ═ ╬ ╣ ║ ╗ ╝
 *      THE WISDOM!
 *  T H E   W I S D O M !
 *                   ║
 *           ╔═══════╣
 *           ║  ╔════╣
 *           ║  ║    ║
 *           ║  ║    ║
 *                   ║
 *  **/
}

