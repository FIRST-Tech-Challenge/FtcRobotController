package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    //drive chassis
    public static double powerFactor = 0.4;

    //Deposit
    public static double deposit_Slide_down_Pos = 50;   //slides Position Configure
    public static double deposit_Slide_Highbar_Pos = 795;  //slides Position Configure
    public static double deposit_Slide_Highbasket_Pos = 3000; //slides Position Configure

    public static double deposit_Wrist_dump_Pos = 0.3;
    public static double deposit_Wrist_retract_Pos = 0;

    public static double deposit_Arm_dump_Pos = 0.8;
    public static double deposit_Arm_retract_Pos = 0.05;

    public static double deposit_Arm_hook_Pos = 0.8;
    public static double deposit_Wrist_hook_Pos = 0.3;

    public static double dumpTime = 1.5;
    public static double retractTime = 2.8;

    public static double deposit_Slide_UpLiftPower = 0.7;  //slides power
    public static double downLiftPower = 0.5;  //slides power

    //Intake Parameter Configure
    public static double intake_Idle = 0.3;
    public static double intake_Dump = 0.0;

    public static double intake_Slide_Initial   = 0.4;
    public static double intake_slide_Extension = 0.65;
    public static double intake_slide_Retract   = 0.4;

    public static double intake_Rotation        = 0.4;

    public static double intake_Arm_initial     = 0.4;
    public static double intake_Arm_down        = 0.1;
    public static double intake_Arm_retract     = 0.55;

    public static double intake_Claw_Open       = 0.1;
    public static double intake_Claw_Close      = 0.2;
}
