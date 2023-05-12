package org.firstinspires.ftc.teamcode.commandBased;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.classes.Vector2d;

@Config
public class Constants {

    //DRIVE CONSTANTS
    public static double TRACK_WIDTH = 13;
    public static double DRIVE_KV = 0;
    public static PIDCoefficientsEx TURN_COEFFS = new PIDCoefficientsEx(1, 0.2, 0.2, 1, 1, 0);
    public static Vector2d TARGET= new Vector2d(5, 5);

    //ELEVATOR CONSTANTS
    public static PIDCoefficientsEx elevatorCoeffsEx = new PIDCoefficientsEx(0.01, 0, 0, 0.25, 2, 0.5);;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.0075, 0, 0.00005);

    public static double eleKg = 0.05;
    public static double elekV = 0;
    public static double elekA = 0;
    public static double elekStatic = 0;

    public static double eleMaxVel = 200;
    public static double eleMaxAccel = 250;
    public static double eleMaxJerk = 300;

    public static double eleLow = 0;
    public static double eleHigh = 15;

}
