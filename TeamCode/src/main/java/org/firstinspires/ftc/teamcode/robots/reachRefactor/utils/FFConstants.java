package org.firstinspires.ftc.teamcode.robots.reachRefactor.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class FFConstants {

    //----------------------------------------------------------------------------------------------
    // Control Constants
    //----------------------------------------------------------------------------------------------

    public static PIDCoefficients DRIVE_PID_COEFFICIENTS = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients ROTATE_PID_COEFFICIENTS = new PIDCoefficients(0.0055, 0, .13);
    public static PIDCoefficients SWIVEL_PID_COEFFICIENTS = new PIDCoefficients(0.1, 0, 0);

    public static double EPSILON = 0.001;
    public static double TRIGGER_DEADZONE = 0.2;
    public static double FORWARD_SCALING_FACTOR = 1;
    public static double ROTATE_SCALING_FACTOR = 1;

    // tele-op smoothing
    public static double FRONT_LEFT_SMOOTHING_FACTOR = 0.1;
    public static double FRONT_RIGHT_SMOOTHING_FACTOR = 0.1;
    public static double MIDDLE_SMOOTHING_FACTOR = 0.1;

    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    public static double WHEEL_RADIUS = 0.9;
    public static double TRACK_WIDTH = 0.308162;
    public static double DRIVETRAIN_TICKS_PER_REVOLUTION = MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getTicksPerRev();
    public static double DRIVETRAIN_MAX_TICKS_PER_SECOND = MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getAchieveableMaxTicksPerSecond();
    public static double INCHES_PER_METER = 39.3701;
    public static double DRIVETRAIN_METERS_PER_TICK = 2 * Math.PI * WHEEL_RADIUS / DRIVETRAIN_TICKS_PER_REVOLUTION;

    //----------------------------------------------------------------------------------------------
    // Dashboard Constants
    //----------------------------------------------------------------------------------------------

    public static String STROKE_COLOR = "#4D934D";

    //----------------------------------------------------------------------------------------------
    // Miscellaneous
    //----------------------------------------------------------------------------------------------

    public static String[] GAME_STATES = new String[] {"Tele-Op", "Autonomous"};
    public enum Alliance {
        RED, BLUE
    }
    public static String DEFAULT_TELEMETRY_LINE = "Master";
    public static double AVERAGE_LOOP_TIME_SMOOTHING_FACTOR = 0.1;
    public static boolean DEFAULT_DASHBOARD_ENABLED = true;

}
