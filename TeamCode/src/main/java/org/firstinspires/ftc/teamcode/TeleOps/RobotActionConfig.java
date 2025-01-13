package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    //drive chassis
    public static double powerFactor = 1;

    //Intake Configure
    public static double intake_Slide_Extension         = 0.33;// range(0.1 - 0.65)
    public static double intake_Slide_Retract           = 0.0;

    public static double intake_Rotation_Mid            = 0.46; // range(0-1, 0.49 at the middle for installation

    public static double intake_Arm_Initial             = 0.12; //initial arm position, range(0-0.56, 0: lowest, 0.56:fully retracted).
    public static double intake_Arm_Pick                = 0.41; //intake arm pick pos
    public static double intake_Arm_Idle                = 0.25; // intake arm lift a bit before retract
    public static double intake_Arm_Transfer            = 0.1;  // intake arm transfer pos

    public static double intake_Wrist_Initial           = 0.0; /** Needs change**/
    public static double intake_Wrist_Idle              = 0.1; // Final
    public static double intake_Wrist_Pick              = 0.64; // Final
    public static double intake_Wrist_Transfer          = 0.11; // Final
    public static double intake_Rotation_Steer_Amount   = 0.15;

    public static double intake_Claw_Open               = 0.0; //range(0.0 - 0.27)
    public static double intake_Claw_Close              = 0.27;

    //Deposit Slide Config
    public static int deposit_Slide_down_Pos            = 50;   //slides Position Configure
    public static int deposit_Slide_Highbar_Up_Pos      = 1050;//slides Position Configure
    public static int deposit_Slide_Highbasket_Pos      = 3222; //slides Position Configure
    public static int deposit_Slide_Hang_Pos            = 3525;
    //deposit slide power
    public static double deposit_Slide_UpLiftPower      = 1.0;  //slides power
    public static double deposit_Slide_DownLiftPower    = 0.7;

    //Deposit Wrist config
    public static double deposit_Wrist_dump_Pos         = 0.22;
    public static double deposit_Wrist_Transfer_Pos     = 0.53;
    public static double deposit_Wrist_Highbar_Pos      = 0.64;
    public static double deposit_Wrist_PickUp_Pos       = 0.38;
    public static double deposit_Wrist_Flat_Pos         = 0.3;

    //deposit arm config
    public static double deposit_Arm_dump_Pos           = 0.7;
    public static double deposit_Arm_Transfer_Pos       = 0.0;
    public static double deposit_Arm_hang_Pos           = 0.8;
    public static double deposit_Arm_Highbar_Pos        = 0.9;
    public static double deposit_Arm_PickUp_Pos         = 1;

    //deposit claw config
    public static double deposit_Claw_Open              = 0.36;
    public static double deposit_Claw_Close             = 0.08;

    public static float hsvValues[] = {0F,0F,0F};

    // Time
    public static double DEBOUNCE_THRESHOLD                 =0.25; // debounce_Threshold
    // control time
    public static double intake_Claw_Extension_Threshold    = 0.2;
    public static double intake_Claw_Grab_Threshold         = 0.1;
    public static double intake_Slide_Retract_Threshold     = intake_Claw_Grab_Threshold * 2.5;
    public static double intake_Wrist_Arm_Retract_Threshold = intake_Slide_Retract_Threshold + intake_Claw_Extension_Threshold;
    public static double deposit_Claw_Close_Threshold       = 0.8;
    public static double intake_Claw_Open_Threshold         = 0.15;
    public static double intake_Arm_Idle_Threshold          = 1.5;


    public static double depositDumpTime                       = 0.5; // deposit time need to rotate deposit arm then open claw
    public static double depositRetractTime                    = 1.5; // wait then retract slide
    public static double depositPostDumpTime                   = depositDumpTime+0.4;
    public static double transferTime                          = 0.5; // wait for transfer time then open intake claw.
    public static double hookTime                              = 0.3; // wait then release deposit claw.

}
