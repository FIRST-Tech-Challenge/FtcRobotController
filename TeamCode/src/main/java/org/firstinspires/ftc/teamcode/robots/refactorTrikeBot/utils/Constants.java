package org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class Constants {

    //----------------------------------------------------------------------------------------------
    // Control Constants
    //----------------------------------------------------------------------------------------------

    public static PIDCoefficients DRIVE_PID_COEFFICIENTS = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients ROTATE_PID_COEFFICIENTS = new PIDCoefficients(0.0055, 0, .13);
    public static PIDCoefficients SWIVEL_PID_COEFFICIENTS = new PIDCoefficients(0.1, 0, 0);

    public static double EPSILON = 0.001;
    public static double TRIGGER_DEADZONE = 0.2;

    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    public static double WHEEL_RADIUS = 0.9;
    public static double TRACK_WIDTH = 0.308162;
    public static double DRIVETRAIN_TICKS_PER_REVOLUTION = MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getTicksPerRev();
    public static double DRIVETRAIN_MAX_TICKS_PER_SECOND = MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getAchieveableMaxTicksPerSecond();
}
