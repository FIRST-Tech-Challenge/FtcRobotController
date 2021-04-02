package org.firstinspires.ftc.team8923_2020;


public class Constants {

    static final double GEAR_RATIO  = 1.0/1.0;
    static final double TICKS_PER_MOTOR_REVOLUTION = 560.0;
    static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION / GEAR_RATIO;
    static final double WHEEL_DIAMETER = 4 * 25.4; // 4 inch diameter
    static final double MM_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double MM_PER_TICK = MM_PER_REVOLUTION / TICKS_PER_WHEEL_REVOLUTION;
    static final double COUNTS_PER_MM = TICKS_PER_WHEEL_REVOLUTION / MM_PER_REVOLUTION;

    // Controls
    public static final double MINIMUM_JOYSTICK_PWR = 0.0;
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
