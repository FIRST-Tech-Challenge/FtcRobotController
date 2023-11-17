package org.firstinspires.ftc.team6220_CENTERSTAGE;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    /* Constants used in TeleOp */

    public static final double TURN_STICK_DEADZONE = 0.01;
    public static final double TURN_POWER_MULTIPLIER = 1.0;
    public static final double DRIVE_POWER_X_MULTIPLIER = 1.0;
    public static final double DRIVE_POWER_Y_MULTIPLIER = 0.7;
    public static final double TELEOP_MIN_HEADING_ACCURACY = 5.0; // degrees off from target
    public static final double SLOWMODE_MULTIPLIER = 0.3;
    public static final double INTAKE_POWER_MULTIPLIER = 0.8;



    /* Constants used in AutoFramework */

    // /!\ old x and y factor calibration was for incorrect value of 537.7;
    public static final double TICKS_PER_REVOLUTION = 537.6;
    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_DIAMETER = 3.78; // inches
    public static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    public static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    public static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;

    // /!\ x and y factors are outdated; was made using incorrect ticks per revolution
    public static final double AUTO_X_FACTOR = 1.0867924528301887;
    public static final double AUTO_Y_FACTOR = 0.9411764705882353;
    public static final double ROBOT_AUTO_SPEED = 0.5;

    // degrees off from target
    public static final double AUTO_MIN_HEADING_ACCURACY = 5.0;

}
