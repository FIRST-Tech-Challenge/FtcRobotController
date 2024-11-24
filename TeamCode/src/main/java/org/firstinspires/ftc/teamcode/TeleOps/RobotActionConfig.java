package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    //drive chassis
    public static double powerFactor = 0.4;

    public static double intake_slide_Extension = 0.6;// range(0.3 - 0.65)
    public static double intake_slide_Retract = 0.3;

    public static double intake_Rotation = 0.49;

    public static double intake_Arm_initial = 0.1;//0-0.56
    public static double intake_Arm_down = 0.05;
    public static double intake_Arm_retract = 0.53;

    public static double intake_Claw_Open = 0.55;
    public static double intake_Claw_Close = 0.3;

    //Deposit Config
    public static int deposit_Slide_down_Pos = 50;   //slides Position Configure
    public static int deposit_Slide_Highbar_Pos = 795;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos = 2800; //slides Position Configure

    public static double deposit_Wrist_dump_Pos = 0.3;
    public static double deposit_Wrist_retract_Pos = 0.1;

    public static double deposit_Arm_dump_Pos = 0.8;
    public static double deposit_Arm_retract_Pos = 0.0;

    public static double deposit_Arm_hook_Pos = 0.8;
    public static double deposit_Claw_Open = 0.11;
    public static double deposit_Claw_Close = 0.0;

    public static double dumpTime = 1.8;
    public static double retractTime = 3.2;

    public static double deposit_Slide_UpLiftPower = 0.9;  //slides power
    public static double downLiftPower = 0.3;  //slides power
}
