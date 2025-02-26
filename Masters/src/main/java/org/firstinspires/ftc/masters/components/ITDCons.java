package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class ITDCons {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public enum Color {red, blue, yellow, unknown}

    public static double zero = 0;

    public static double intakeInit = 0.5;
    public static double intakeInitLeft = 0;
    public static double intakeInitRight = 1;
    public static double intakeArmDrop =0.80;
    public static double intakeChainDrop = 0.32;

    public static double intakeArmNeutral= 0.80;
    public static double intakeChainNeutral=0.58;

    public static double intakeArmTransfer=0.25;
    public static double intakeChainTransfer = 0.64;
    public static double intakeTransferSpeed = 0.65;
    public static double intakeEjectSpeed =0.8;

    public static double pushIn=0.6;
    public static double pushOut=0.2;

    public static double clawOpenTransfer =0.3;
    public static double clawOpen = 0.35;
    public static double clawClose = 0.75;

    public static double wristFront= 0.06;
    public static double wristBack = 0.75;

    public static double positionBack = 0.09;
    public static double positionInitSpec=0.4;
    public static double positionTransfer = 0.36;
    public static double positionFront =1;

    public static double angleBack = 0.04;
    public static double angleTransfer = 0.69;

    public static double angleMiddle = 0.35;
    public static double angleScoreSpec = 0.45;
    public static double angleScoreSample = 0.15;

    public static int wallPickupTarget = 5500;
//    public static int transferPickupTarget = 0;

    public static int BucketTarget = 60000;
    public static int LowBucketTarget = 30000;
    public static int SpecimenTarget = 21500;

//    public static int TransferPickupTarget = 4800;
//    public static int TransferWaitTarget = 5000;

    public static int intermediateTarget = 20000;
    public static int WallTarget = 4800;

    public static int TransferTarget = 7500;

    public static int MaxExtension = 31000;
    public static int halfExtension= 15000;

    public static int TransferExtension = 0;
    public static int MinExtension = 500;


    //led values
    public static double yellow = 0.388;
    public static double blue = 0.611;
    public static double red = 0.279;
    public static double green = 0.500;
    public static double off =0;

    public static double intakeintakearm = .503;
    public static double intakeintakechain = .08;

    public static int clawOpenWaitTime = 300;
    public static int clawCloseWaitTime= 400;

}

/*

   ╚ ╔ ╩ ╦ ╠ ═ ╬ ╣ ║ ╗ ╝
       THE WISDOM!
   T H E   W I S D O M !
                    ║
            ╔═══════╣
            ║  ╔════╣
            ║  ║    ║
            ║  ║    ║
                    ║

 */

// Wall Hanging Grab Diffy 1 = .17, Diffy 2 = .65
// Floor Grab Diffy 1 = .27, Diffy 2 = .34

// Bucket Diffy 1 = .58, Diffy 2 = .65, Slide Target = 50000
// Specimen Diffy 1 = 0.015, Diffy 2 = 0.815, Slide Target = 18800