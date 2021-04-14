package org.firstinspires.ftc.team8923_2020;

public class Constants {

    //ratios
    static final double TICKS_PER_REVOLUTION = 537.6; // Neverest orbital 20, 7 pulse per revolution
    static final double GEAR_RATIO = 1.0;
    static final double WHEEL_DIAMETER = 4.0; // inches
    static final double TICKS_PER_INCH =  (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;

    //controls
    public static final double MINIMUM_JOYSTICK_POWER = 0.0;
    public static final double MINIMUM_TRIGGER_VALUE = 0.33;
    public static final double MINIMUM_DRIVE_POWER = 0.08;

    //Move Auto Constants
    public static final double ROTATION_P = 0.05;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final double TRANSLATION_P = 0.0004;
    public static final double TRANSLATION_I = 0.0;
    public static final double TRANSLATION_D = 0.0;
    public static final double ANGLE_TOLERANCE_DEG = 5.0;
    public static final double POSITION_TOLERANCE_MM = 2*25.4;

    //Intake
    public static final double INTAKE_PWR = 1;

    //Lift
    public static final double LIFT_PWR = 1;

    //Shooter
    public static final double SHOOTER_PWR = 1;

}
