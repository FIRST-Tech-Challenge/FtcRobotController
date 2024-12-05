package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
public class ITDCons {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double zero = 0;
    public static double slideInit = 0.5;
    public static double liftInit = 0.9;
    public static double liftUp = 0.9;
    public static double liftDown = 0.1;

    public static double open = 0.35;
    public static double close = 0.7;

    public static double FloorDiffy1 = .27;
    public static double FloorDiffy2 = .34;

    public static double WallDiffy1 = .065;
    public static double WallDiffy2 = .645;

    public static double BucketDiffy1 = .58;
    public static double BucketDiffy2 = .65;
    public static int BucketTarget = 50000;

    public static double SpecimenDiffy1 = .065;
    public static double SpecimenDiffy2 = .645;
    public static int SpecimenTarget = 32000;

    public static int ReleaseTarget = 28000;


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