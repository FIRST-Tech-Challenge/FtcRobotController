/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Generic configuration
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/* System includes */
import java.util.HashMap;
import java.util.Map;

abstract public class HMapConfig {

    public static HMapConfig s_Current = new V1();

    protected String  FRONT_LEFT_WHEEL = "";
    protected String  BACK_LEFT_WHEEL = "";
    protected String  FRONT_RIGHT_WHEEL = "";
    protected String  BACK_RIGHT_WHEEL = "";
    protected boolean FRONT_LEFT_WHEEL_REVERSE = false;
    protected boolean BACK_LEFT_WHEEL_REVERSE = false;
    protected boolean FRONT_RIGHT_WHEEL_REVERSE = false;
    protected boolean BACK_RIGHT_WHEEL_REVERSE = false;


    protected String  INTAKE_SLIDES = "";
    protected boolean INTAKE_SLIDES_REVERSE;
    protected String  INTAKE_ARM_PITCH_LEFT = "";
    protected String  INTAKE_ARM_PITCH_RIGHT = "";
    protected String  INTAKE_ELBOW_PITCH = "";
    protected String  INTAKE_WRIST_ROLL = "";
    protected String  INTAKE_CLAW = "";


    protected String  OUTTAKE_SLIDES_LEFT = "";
    protected String  OUTTAKE_SLIDES_RIGHT = "";
    protected boolean OUTTAKE_SLIDES_LEFT_REVERSE = false;
    protected boolean OUTTAKE_SLIDES_RIGHT_REVERSE = false;
    protected String  OUTTAKE_ELBOW_PITCH_RIGHT = "";
    protected String  OUTTAKE_ELBOW_PITCH_LEFT = "";
    protected String  OUTTAKE_WRIST_ROLL = "";
    protected String  OUTTAKE_CLAW = "";


    protected String  BUILT_IN_IMU = "";
    protected RevHubOrientationOnRobot.LogoFacingDirection BUILT_IN_IMU_LOGO;
    protected RevHubOrientationOnRobot.UsbFacingDirection  BUILT_IN_IMU_USB;
    protected String  OTOS = "";


    // Abstract method for initializing specific configurations
    protected abstract void initialize();

    // Public getters

    public String  FRONT_LEFT_WHEEL()             { return FRONT_LEFT_WHEEL; }
    public String  BACK_LEFT_WHEEL()              { return BACK_LEFT_WHEEL; }
    public String  FRONT_RIGHT_WHEEL()            { return FRONT_RIGHT_WHEEL; }
    public String  BACK_RIGHT_WHEEL()             { return BACK_RIGHT_WHEEL; }

    public String INTAKE_SLIDES()                 { return INTAKE_SLIDES; }
    public String INTAKE_ARM_PITCH_LEFT()         { return INTAKE_ARM_PITCH_LEFT; }
    public String INTAKE_ARM_PITCH_RIGHT()        { return INTAKE_ARM_PITCH_RIGHT; }
    public String INTAKE_ELBOW_PITCH()            { return INTAKE_ELBOW_PITCH; }
    public String INTAKE_WRIST_ROLL()             { return INTAKE_WRIST_ROLL; }
    public String INTAKE_CLAW()                   { return INTAKE_CLAW; }

    public String  OUTTAKE_SLIDES_LEFT()          { return OUTTAKE_SLIDES_LEFT; }
    public String  OUTTAKE_SLIDES_RIGHT()         { return OUTTAKE_SLIDES_RIGHT; }
    public String  OUTTAKE_ELBOW_PITCH_RIGHT()    { return OUTTAKE_ELBOW_PITCH_RIGHT; }
    public String  OUTTAKE_ELBOW_PITCH_LEFT()     { return OUTTAKE_ELBOW_PITCH_LEFT; }
    public String  OUTTAKE_WRIST_ROLL()           { return OUTTAKE_WRIST_ROLL; }
    public String  OUTTAKE_CLAW()                 { return OUTTAKE_CLAW; }

    public String  BUILT_IN_IMU()                 { return BUILT_IN_IMU; }
    public String  OTOS()                         { return OTOS; }

    public boolean FRONT_LEFT_WHEEL_REVERSE()     { return FRONT_LEFT_WHEEL_REVERSE; }
    public boolean BACK_LEFT_WHEEL_REVERSE()      { return BACK_LEFT_WHEEL_REVERSE; }
    public boolean FRONT_RIGHT_WHEEL_REVERSE()    { return FRONT_RIGHT_WHEEL_REVERSE; }
    public boolean BACK_RIGHT_WHEEL_REVERSE()     { return BACK_RIGHT_WHEEL_REVERSE; }

    public boolean INTAKE_SLIDES_REVERSE()        { return INTAKE_SLIDES_REVERSE; }
    public boolean OUTTAKE_SLIDES_LEFT_REVERSE()  { return OUTTAKE_SLIDES_LEFT_REVERSE; }
    public boolean OUTTAKE_SLIDES_RIGHT_REVERSE() { return OUTTAKE_SLIDES_RIGHT_REVERSE; }

    public RevHubOrientationOnRobot.LogoFacingDirection BUILT_IN_IMU_LOGO() { return BUILT_IN_IMU_LOGO; }
    public RevHubOrientationOnRobot.UsbFacingDirection  BUILT_IN_IMU_USB()  { return BUILT_IN_IMU_USB; }

    // Constructor
    public HMapConfig() { initialize(); }
}
