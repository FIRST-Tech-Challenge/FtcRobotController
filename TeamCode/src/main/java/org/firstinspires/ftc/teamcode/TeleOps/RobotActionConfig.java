package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    //drive chassis
    public static double powerFactor = 0.4;

    //Intake Configure
    public static double intake_slide_Extension         = 0.6;// range(0.3 - 0.65)
    public static double intake_slide_Retract           = 0.3;

    public static double intake_Rotation                = 0.49; // range(0-1, 0.49 at the middle for installation

    public static double intake_Arm_Initial             = 0.30;//range(0-0.56, 0: lowest, 0.56:fully retracted).
    public static double intake_Arm_Pick                = 0.05;
    public static double intake_Arm_Idle                = 0.10;
    public static double intake_Arm_Transfer            = 0.53;

    public static double intake_Claw_Open               = 0.55; //range(0.25 - 0.6)
    public static double intake_Claw_Close              = 0.3;

    //Deposit Config
    public static int deposit_Slide_down_Pos             = 50;   //range (0-3300), 50 to prevent hard hit.
    public static int deposit_Slide_Highbar_Pos         = 795;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos      = 2800; //highest point

    public static double deposit_Wrist_dump_Pos         = 0.3; //range(0-1), 0: installation position
    public static double deposit_Wrist_retract_Pos      = 0.06; // 0.06 is rest position.

    public static double deposit_Arm_dump_Pos           = 0.8; // range (0-1) 0: installation position
    public static double deposit_Arm_retract_Pos        = 0.0; // 0 is rest position.

    public static double deposit_Arm_hook_Pos           = 0.8;
    public static double deposit_Claw_Open              = 0.11;
    public static double deposit_Claw_Close             = 0.0;

    public static double dumpTime                       = 1.8; // deposit time need to rotate deposit arm then open claw
    public static double retractTime                    = 3.2; // wait then retract slide

    public static int deposit_Slide_UpLiftPower         = 1;  //slides power
    public static double downLiftPower                  = 0.7;  //slides power

}
