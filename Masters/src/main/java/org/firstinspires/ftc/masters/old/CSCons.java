package org.firstinspires.ftc.masters.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Config
public class CSCons {

    public static double skewampus = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double pixelDetectThreshold = 350;
    public static double clawOpen = 0.2;
    public static double clawClosed = 0.57;
    public static double clawTransfer = 0.38; // .65
    public static double clawAngleGroundToThree = 1; //.9?
    public static double clawAngleFourToFive = .98;
    public static double clawAngleTransition = .34;
    public static double clawAngleTransfer = .12;
    public static double clawArmGround = 0.82;//.78;
    public static double clawArm2 = .75;
    public static double clawArm3 = .72;
    public static double clawArm4 = .65;
    public static double clawArm5 = .62;
    public static double clawArmTransition = .3;
    public static double clawArmTransfer = .3;
//    public static double clawArmTransition = .23;
//    public static double clawArmTransfer = .24;

    //public static double[]outtakeAngle={0.234, 0.9}; //folder, transfer
    public static double outtakeAngleFolder=0;

    public static double outtakeAngleTransfer=1;
    //test public static double outtakeAngleTransfer=.53;
    //public static double[] doubleServoBack= {0.7, 0.15}; //drop, transfer
    public static double outtakeMovementBackDrop =0;
    public static double outtakeMovementTransfer =0.55;//0.55

    public static double wristAngleInit= 0.9;
    public static double wristOuttakeAngleBackdrop =0.04;
    public static double wristOuttakeAngleTransfer =0.98;
    public static double wristOuttakeMovementTransfer=0.43; //0.43

    public static double wristOuttakeMovementM=0.6;

    public static double wristOuttakeMovementGround = 0.89;
    public static double wristOuttakeMovementHub = 1;

    public static double wristOuttakePickup=0.37;
    public static double wristOuttakeAnglePickup=0.95;
    public static double wristOuttakeMovementBackdrop =0.82;
    //public static double[] rightSideBack = {0.955, 0.65};
    public static double openHook = 0.6;
    public static double closeHook = 1;
    public static double openMicroHook =0.5;
    public static double closeMicroHook =0;

    public static double wristFlatRight=0.17; //
    public static double wristAngleRight=0.35; //
    public static double wristVertical= 0.46; //
    public static double wristAngleLeft =0.55; //
    public static double wristFlatLeft =0.745; //
    public static double wristVerticalDown=1;

    public static double droneFlat = 0.5;
    public static double droneShooting = 0.11;

    public static double closeClawDistance = 3; //in cm

    public static long transferToBottomIntake = 200; //time in ms
    public static long transferToScoreOuttake = 200;
    public static long scoreToTransferOuttake = 200;
    public static long closingHook = 100;

    public static int leftIntakeExtension = 990;
    public static int centerIntakeExtension = 1240;
    public static int rightIntakeExtension = 620;

    public static int redRightIntakeExtension= 660;
    public static int redCenterIntakeExtension = 1240;
    public static int getLeftIntakeExtension = 960;


    public static double backMultiplier =0.78;
    public static double frontMultiplier = 1;

    public static double servo1Up = 0.7;
    public static double servo1Down = 0.35;
    public static double servo2Up =0.3;
    public static double servo2Down= 0.65;

    public static double wristCenter = 0.5;

    public static double tagBackboardX = 63;
    public static double tag1Y = 42;
    public static double tag2Y = 35.5;
    public static double tag3Y = 29;
    public static double tag4Y = -29;
    public static double tag5Y = -35.5;
    public static double tag6Y = -42;

    public static double tagAudienceX = -72;
    public static double tag9Y= 36;
    public static double tag10Y= 41.5;
    public static double tag7Y = -41.5;
    public static double tag8Y = -36;

    public static double cameraOffsetY= 0;
    public static double cameraOffsetX = -9;
    public static double cameraFrontOffsetY= -6;
    public static double cameraFrontOffsetX=9;


    public static double intakeGround = 0.38; //for autonomous, purple pixel
    public static double intakeBottom = 0.33;
    public static double intake2 =0.31;
    public static double intake3 =0.29;
    public static double intake4 =0.27;
    public static double intake5 =0.24;
    public static double intakeAboveTop =0.2;
    public static double intakeInit =0.02;

    public static double speed =-1;

    public static double transferUp = 0.1;
    public static double transferPush = 0.58;


    public enum OuttakePosition{
        BOTTOM (-10),
        LOW_AUTO(105),
        AUTO(800),
        LOW(800),
        MID(1500),
        HIGH(2400);

        private int target;

        private OuttakePosition(int target){
            this.target = target;
        }

        public int getTarget(){
            return  target;
        }
    }

    public enum IntakeState {
        Transition, Transfer, Intake, MoveToIntake, MoveToTransfer;
    }

    public enum OuttakeState{
        ClosingHook, MoveToTransfer, ReadyToTransfer, MoveToDrop, ReadyToDrop, Align, BackUp, GrabPixels, Retract
    }

    public enum DriveMode {
        NORMAL,
        PIXEL_SCORE,
        END_GAME,
        HANG
    }

    public enum ClawPosition{
        OPEN, CLOSED, TRANSFER
    }

    public enum  HookPosition{
        OPEN, CLOSED, HALF
    }

    public enum IntakeDirection {
        ON, OFF, BACKWARD
    }

    public enum OuttakeWrist{
        flatRight (CSCons.wristFlatRight), angleRight(CSCons.wristAngleRight), verticalDown(CSCons.wristVerticalDown),
        vertical(CSCons.wristVertical), angleLeft(CSCons.wristAngleLeft), flatLeft(CSCons.wristFlatLeft);

        double position= -1;
        private OuttakeWrist(double position){
            this.position= position;
        }

        public double getPosition(){
            return position;
        }
    }

    public enum TransferStatus {
        WAITING_FOR_PIXELS (100),
        MOVE_ARM (100),
        MOVE_OUTTAKE(100),
        CLOSE_FINGERS (500),
        DONE(0);

        private int waitTime;
        private TransferStatus(int waitTime){
            this.waitTime = waitTime;
        }

        public int getWaitTime() {
            return waitTime;
        }
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

