package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    //drive chassis
    public static double powerFactor = 1;

    //Intake Configure
    public static double intake_Slide_Extension         = 0.36;// range(0.1 - 0.65)
    public static double intake_Slide_Retract           = 0.1;

    public static double intake_Rotation_Mid            = 0.46; // range(0-1, 0.49 at the middle for installation

    public static double intake_Arm_Initial             = 0.12; //initial arm position, range(0-0.56, 0: lowest, 0.56:fully retracted).
    public static double intake_Arm_Pick                = 0.415; //intake arm pick pos
    public static double intake_Arm_Idle                = 0.25; // intake arm lift a bit before retract
    public static double intake_Arm_Transfer           = 0.1;  // intake arm transfer pos

    public static double intake_Wrist_Initial           = 0.0; /** Needs change**/
    public static double intake_Wrist_Idle              = 0.6; /** Needs change**/
    public static double intake_Wrist_Pick              = 0.64; /** Needs change**/
    public static double intake_Wrist_Transfer          = 0.07; /** Needs change**/

    public static double intake_Claw_Open               = 0.0; //range(0.0 - 0.27)
    public static double intake_Claw_Close              = 0.27;

    //Deposit Config
    public static int deposit_Slide_Down_Pos             = 100;   //range (0-3300), 50 to prevent hard hit.
    public static int deposit_Slide_Highbar_Pos         = 1670;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos      = 3222; //highest point

    public static double deposit_Wrist_Dump_Pos         = 0.22; //range(0-1), 0: installation position
    public static double deposit_Wrist_Transfer_Pos      = 0.53; // 0.06 is rest position.
    public static double deposit_Wrist_Pick_Pos         = 0.31; // 0.06 is rest position.
    public static double deposit_Wrist_Hook_Pos         = 0.5; // 0.06 is rest position.


    public static double deposit_Arm_Pick_Pos           = 0.97; // 0 is pick position.
    public static double deposit_Arm_Dump_Pos           = 0.7; // range (0-1) 0: installation position 180 deg
    public static double deposit_Arm_Transfer_Pos        = 0.06; // 0 is rest position.

    public static double deposit_Arm_Hook_Pos           = 0.5;  // deposit arm hook position
    public static double deposit_Claw_Open              = 0.36;  //
    public static double deposit_Claw_Close             = 0.0;

    public static double dumpTime                       = 0.5; // deposit time need to rotate deposit arm then open claw
    public static double retractTime                    = 1.5; // wait then retract slide

    public static double postDumpTime                   = dumpTime+0.4;
    public static double transferTime                   = 0.5; // wait for transfer time then open intake claw.
    public static double hookTime                       = 0.3; // wait then release deposit claw.


    public static double deposit_Slide_UpLiftPower      = 1.0;  //slides power
    public static double deposit_Slide_DownLiftPower    = 0.7;  //slides power
    public static double DEBOUNCE_THRESHOLD             =0.25; // debounce_Threshold

}
