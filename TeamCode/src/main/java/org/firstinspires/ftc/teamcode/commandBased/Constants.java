package org.firstinspires.ftc.teamcode.commandBased;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.commandBased.classes.Vector2d;

@SuppressWarnings("SpellCheckingInspection")
@Config
public class Constants {

    //debug toggles
    public static boolean DEBUG_DRIVE = false;
    public static boolean DEBUG_ELE = false;
    public static boolean DEBUG_ARM = false;
    public static boolean DEBUG_ROTATOR = false;
    public static boolean DEBUG_INTAKE = true;
    public static boolean DEBUG_COMMANDS = false;

    //display toggle
    public static boolean DISPLAY = true;

    //control layers
    public static GamepadKeys.Trigger CONTROL_LAYER_2 = GamepadKeys.Trigger.LEFT_TRIGGER;
    public static GamepadKeys.Trigger CONTROL_LAYER_3 = GamepadKeys.Trigger.RIGHT_TRIGGER;

    //motor types
    public static Motor.GoBILDA DRIVE_MOTOR = Motor.GoBILDA.RPM_312;
    public static Motor.GoBILDA ELE_MOTOR = Motor.GoBILDA.RPM_312;
    public static Motor.GoBILDA ARM_MOTOR = Motor.GoBILDA.RPM_117;

    //drivetrain constants
    public static double DRIVE_FAST_STRAFE = 1;
    public static double DRIVE_FAST_FORWARD = 1;
    public static double DRIVE_FAST_TURN = 1;

    public static double DRIVE_SLOW_STRAFE = 0.5;
    public static double DRIVE_SLOW_FORWARD = 0.5;
    public static double DRIVE_SLOW_TURN = 0.5;

    public static double TRACK_WIDTH = 13;
    public static double DRIVE_KV = 0;
    public static PIDCoefficientsEx TURN_COEFFS = new PIDCoefficientsEx(1, 0.2, 0.2, 1, 0, 0);
    public static Vector2d TARGET = new Vector2d(-10, 0);
    public static Pose2d STARTING_POINT = new Pose2d(0, 0, Math.toRadians(0));
    public static double ANGLE_OFFSET = 180;

    //elevator pid
    //public static PIDCoefficients ELE_COEFFS = new PIDCoefficients(0.0075, 0, 0.00005);
    public static PIDCoefficients ELE_COEFFS = new PIDCoefficients(0.005, 00.0002, 0.0004);
    public static double ELE_KG = 0.175;
    public static double ELE_KV = 0;
    public static double ELE_KA = 0;
    public static double ELE_KS = 0;

    //elevator motion profile
    public static double ELE_MAX_VEL = 200;
    public static double ELE_MAX_ACCEL = 50;

    //elevator positions
    public static double ELE_LOW = 0;
    public static double ELE_IDLE = 2;
    public static double ELE_STACK = 5;
    public static double ELE_MID = 9;
    public static double ELE_HIGH = 12;

    //command-ending deadzones
    public static double ELE_DONE_DEADZONE = 1;
    public static double ELE_TRIGGER = 6;

    //arm pid
    public static PIDCoefficients ARM_COEFFS = new PIDCoefficients(0.01, 0.0001, 0.0001);

    public static double ARM_KV = 0.001;
    public static double ARM_KA = 0;
    public static double ARM_KS = 0.075;
    public static double ARM_KSIN = 0.125;

    public static double ARM_VELO_FRONT_BACK = 1600;
    public static double ARM_ACCEL_FRONT_BACK = 4000;

    public static double ARM_VELO_IDLE_FRONT = 1600;
    public static double ARM_ACCEL_IDLE_FRONT = 3000;

    public static double ARM_VELO_IDLE_BACK = 1600;
    public static double ARM_ACCEL_IDLE_BACK = 3200;

    public static double ARM_VELO_FRONT_IDLE = 1600;
    public static double ARM_ACCEL_FRONT_IDLE = 3200;

    public static double ARM_VELO_BACK_IDLE = 1600;
    public static double ARM_ACCEL_BACK_IDLE = 3500;

    //arm angle positions
    public static double ARM_ANGLE_BACK = -110;
    public static double ARM_ANGLE_IDLE = 42.5;
    public static double ARM_ANGLE_FRONT = 110;
    public static double ARM_ANGLE_MAX = 130;

    public static double ARM_ANGLE_TRIGGER = 75;
    public static double ARM_ANGLE_DEADZONE = 10;

    //arm encoder positions
    public static double ARM_ENC_BACK_MAX = -720;
    public static double ARM_ENC_BACK_PARALLEL = -520;
    public static double ARM_ENC_FRONT_PARALLEL = 220;
    public static double ARM_ENC_FRONT_MAX = 410;
    public static double ARM_ENC_CENTER = -145;

    //rotator limits
    public static double ROTATOR_FRONT = 1;
    public static double ROTATOR_BACK = 0;
    public static PwmControl.PwmRange TUNED_RANGE = new PwmControl.PwmRange(590, 2400);

    public static double ROTATOR_SMALL_INCREMENT = 10;
    public static double ROTATOR_LARGE_INCREMENT = 100;

    public static int ROTATOR_AVG_LENGTH = 10;

    //intake powers
    public static double INTAKE_IN = 1;
    public static double INTAKE_IDLE = 0;
    public static double INTAKE_OUT = -1;
    public static int INTAKE_AVG_LENGTH = 75;


    //AUTO
    public static double STACK_INCREMENT = .5;

}
