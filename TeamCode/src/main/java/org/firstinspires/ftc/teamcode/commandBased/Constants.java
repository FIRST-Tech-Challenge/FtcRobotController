package org.firstinspires.ftc.teamcode.commandBased;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.classes.Vector2d;

@Config
public class Constants {

    //motor types
    public static Motor.GoBILDA DRIVE_MOTOR = Motor.GoBILDA.RPM_312;
    public static Motor.GoBILDA ELEVATOR_MOTOR = Motor.GoBILDA.RPM_312;
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
    public static PIDCoefficients ELE_COEFFS = new PIDCoefficients(0.0075, 0, 0.00005);
    public static double ELE_KG = 0.05;
    public static double ELE_KV = 0;
    public static double ELE_KA = 0;
    public static double ELE_KS = 0;

    //elevator motion profile
    public static double ELE_MAX_VEL = 200;
    public static double ELE_MAX_ACCEL = 250;

    //elevator positions
    public static double ELE_LOW = 0;
    public static double ELE_MID_LOW = 4;
    public static double ELE_MID_HIGH = 10;
    public static double ELE_HIGH = 15;

    //command-ending deadzones
    public static double ELE_DONE_DEADZONE = 5;

    //arm pid
    public static double ARM_KP = 0;
    public static double ARM_KI = 0;
    public static double ARM_KD = 0;
    public static double ARM_KCOS = 0.1;

    public static double ARM_KP_0 = 0;
    public static double ARM_KP_90 = 0;
    public static double ARM_KI_0 = 0;
    public static double ARM_KI_90 = 0;
    public static double ARM_KD_0 = 0;
    public static double ARM_KD_90 = 0;

    //arm positions
    public static double ARM_FRONT = -100;
    public static double ARM_IDLE = 20;
    public static double ARM_BACK = 100;
    public static double ARM_MAX = 120;

    public static InterpLUT ARM_LUT = new InterpLUT();

    //rotator
    public static double ROTATOR_MIN = 10;
    public static double ROTATOR_MAX =190;
}
