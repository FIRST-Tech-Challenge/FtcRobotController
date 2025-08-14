package org.firstinspires.ftc.teamcode.subsystems.mecanum;

//import com.acmerobotics.dashboard.config.Config;

//@Config // Uncomment if using FTC Dashboard for live tuning
public class MecanumConstants {

    // --- Position PID Constants ---
    public static double kpx = 0.1;
    public static double kdx = 0.00225;
    public static double kix = 0.0;

    public static double kpy = 0.1;
    public static double kdy = 0.00225;
    public static double kiy = 0.0;

    public static double kptheta = 1.7;
    public static double kdtheta = 0.023;
    public static double kitheta = 0.0;

    // --- Cascade PID Constants (Position Layer) ---
    public static double cascadekpx = 0.045;
    public static double cascadekdx = 0.0005;
    public static double cascadekix = 0.0075 / 2;

    public static double cascadekpy = 0.045;
    public static double cascadekdy = 0.0005;
    public static double cascadekiy = 0.004 / 2;

    public static double cascadekptheta = 2;
    public static double cascadekdtheta = 0.05;
    public static double cascadekitheta = 0.0075 / 2;


    // --- SquID Constants ---
    public static double squidX = 0;
    public static double squidY = 0;
    public static double squidHeading = 0;

    // Maximum motor RPM of wheel/motor combination
    public static final double MAX_RPM = 312;
    // Maximum angular velocity in radians per second, calculated from RPM
    public static double MAX_ANGULAR_VEL = MAX_RPM / 30 * Math.PI;

    // variable to assist with the power scaling factor in the motor control
    public static final double POWER_SCALE_FACTOR = 1.0;
}