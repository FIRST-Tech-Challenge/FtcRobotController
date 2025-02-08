package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class ITDCons {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double zero = 0;

    public static double intakeInit = 0.5;
    public static double intakeInitLeft = 0;
    public static double intakeInitRight = 1;
    public static double intakeArmDrop =0.55;
    public static double intakeChainDrop = 0;

    public static double intakeArmTransfer=0.4;
    public static double intakeChainTransfer = 0.45;
    public static double liftIntake =0.5;
    public static double liftIntakeLeft= 0.55;
    public static double liftIntakeRight = 0.45;

    public static double gateOpen=0.6;
    public static double gateClose=0.04;

    public static double clawOpen = 0.35;
    public static double clawClose = 0.75;

    public static double wristFront= 0.2;
    public static double wristBack = 0.77;

    public static double positionBack = 0;
    public static double positionInitSpec=0.4;
    public static double positionTransfer = 0.5;
    public static double positionFront =1;

    public static double angleBack = 0;
    public static double angleFront = 0.5;
    public static double angleMiddle = 0.35;
    public static double angleScore = 0.4;

    public static int wallPickupTarget = 5500;
    public static int transferPickupTarget = 0;

    public static double slideInit = 0.5;
    public static double slideOut=0.4;
    public static double slideIn =0.5;
    public static double liftInit = 0.9;
    public static double liftUp = 0.9;
    public static double liftDown = 0.1;


    public static int BucketTarget = 50000;
    public static int SpecimenTarget = 20000;
    public static int ReleaseTarget = 20000;
    public static int intermediateTarget = 20000;
    public static int WallTarget = 6000;

    public static int MaxExtension = 1500;
    public static int MinExtension = 500;

    //color threshold
    public static int blueMin =0;
    public static int blueMax =100;
    public static int redMin = 0;
    public static int redMax =100;
    public static int yellowMin =0;
    public static int yellowMax =100;

    //led values
    public static double yellow = 0.388;
    public static double blue = 0.611;
    public static double red = 0.279;
    public static double off =0;


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