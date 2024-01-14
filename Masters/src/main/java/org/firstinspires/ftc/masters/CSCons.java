package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class CSCons {

    public static double skewampus = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static int[] backSlidesPos = {0,1725,2400,0,0,0,0,0,0,0,0}; // 11 layers of scoring
    public static double[] gpSlidePos = {0,0,0,0}; // in, thirds to full extension
    public static double[] claw = {0.4,0.82, 0.72}; // open, closed, transfer
    //public static double[] clawAngles = {1,1,0.3,0.225}; // Ground - 3, 4 - 5, TRANSition, TRANSfer
    public static double clawAngleGroundToThree = 1;
    public static double clawAngleFourToFive = 1;
    public static double clawAngleTransition = .3;
    public static double clawAngleTransfer = .225;
    //public static double[] clawArm = {0.885,0.795,0.755,0.73,0.695,0.375,0.375}; // In Order G,2,3,4,5,TRANSition,TRANSfer
    public static double clawArmG = .885;
    public static double clawArm2 = .795;
    public static double clawArm3 = .755;
    public static double clawArm4 = .73;
    public static double clawArm5 = .695;
    public static double clawArmTransition = .375;
    public static double clawArmTransfer = .45;

    //public static double[]outtakeAngle={0.234, 0.9}; //folder, transfer
    public static double outtakeAngleFolder=.33;
    public static double outtakeAngleTransfer=.9;
    //public static double[] doubleServoBack= {0.7, 0.15}; //drop, transfer
    public static double outtakeMovementBackDrop = .625;
    public static double outtakeMovementBackTransfer = .15;
    //public static double[] rightSideBack = {0.955, 0.65};
    public static double[] outtakeHook = {0.309,0.79}; // open, close

    public static double outakeHookOpen = 0.309;
    public static double outakeHookClosed = 0.78;

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
        Transition, Transfer, Intake
    }

    public enum OuttakeState{
        MoveToTransfer, ReadyToTransfer, MoveToDrop, ReadyToDrop, Align, BackUp
    }

    public enum DriveMode {
        NORMAL,
        PIXEL_SCORE,
        END_GAME
    }

/**
 *
 *  ╚ ╔ ╩ ╦ ╠ ═ ╬ ╣ ║ ╗ ╝
 *      THE WISDOM!
 *  T H E   W I S D O M !
 *                          ║
 *           ╔══════╣
 *           ║  ╔════╣
 *           ║  ║         ║
 *           ║  ║         ║
 *                          ║
 *  **/
}

