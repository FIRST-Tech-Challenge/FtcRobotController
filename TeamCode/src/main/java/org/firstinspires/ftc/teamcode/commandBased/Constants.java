package org.firstinspires.ftc.teamcode.commandBased;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.classes.Vector2d;

@Config
public class Constants {

    //drivetrain constants
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

}
