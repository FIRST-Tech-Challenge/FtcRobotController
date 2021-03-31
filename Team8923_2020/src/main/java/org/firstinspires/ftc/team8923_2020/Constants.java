package org.firstinspires.ftc.team8923_2020;

public class Constants {

    //ratios
    static final double TICKS_PER_REVOLUTION = 537.6; // Neverest orbital 20, 7 pulse per revolution
    static final double GEAR_RATIO = 1.0;
    static final double WHEEL_DIAMETER = 4.0; // inches
    static final double TICKS_PER_INCH =  (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);

    //controls
    public static final double MINIMUM_JOYSTICK_POWER = 0.0;
    public static final double MINIMUM_TRIGGER_VALUE = 0.33;

}


