package org.firstinspires.ftc.teamcode.common;

import org.checkerframework.checker.units.qual.C;

public class Constants {
    public double LOOP_ITERATION_TIME = 0.025; //must test later on

    //Drive Train Constants
    public double LOAD_ON = 0.6; //assumption
    public double RPM = 1150 * LOAD_ON; //690
    public double RPS = RPM / 60.0; //11.5 ish motor revolutions per second, with load

    public double CLICKS_PER_REV = 384.5; //clicks per rev of motor
    public double CLICKS_PER_SEC = RPS * CLICKS_PER_REV;
    public double WHEEL_DIAMETER = 92 / 25.4; //unit is in inches
    public double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public double MAX_VELOCITY_DT = 2700; // unit is clicks/sec
    public double clickTOLERANCE = 3; //number of clicks or degrees the robot can be off by
    public double degreeTOLERANCE = 1;

    //Swerve constants
        //module translation
    public double InputShaftREV_PER_WheelREV = 17.0 / 16.0; //assuming other gear isn't messing stuff up, 17 revolutions of the input shaft = 16 revolutions of the wheel
    public double Clicks_PER_WheelREV = InputShaftREV_PER_WheelREV * CLICKS_PER_REV; // ~408.53 clicks per wheel revolution
    public double CLICKS_PER_INCH = Clicks_PER_WheelREV / WHEEL_CIRCUMFERENCE; //~34.406 clicks per inch (assuming other gear isn't messing stuff up)
    public double INCHES_PER_CLICK = 1.0 / CLICKS_PER_INCH; //~0.029 inches per click
    public double INCHES_PER_SECOND = 1 / (InputShaftREV_PER_WheelREV * (1/RPS) * (1/WHEEL_CIRCUMFERENCE)); //~128 inches per second...this is definitely wrong...

    //module rotation
    public double InputShaftREV_PER_BigGearREV = 85.0 / 24.0;
    public double Clicks_PER_BigGearRev = InputShaftREV_PER_BigGearREV * CLICKS_PER_REV; //~361.77 clicks per big gear revolution
    public double CLICKS_PER_DEGREE = Clicks_PER_BigGearRev / 360.0; //~3.78 clicks per degree (assuming other gear isn't messing stuff up)
    public double DEGREES_PER_CLICK = 1.0 / CLICKS_PER_DEGREE;

    public double DEGREES_PER_INCH = CLICKS_PER_INCH * DEGREES_PER_CLICK;
    public double INCHES_PER_DEGREE = 1.0 / DEGREES_PER_INCH;

    public double tableSpinRotPercAllocation = 0.5;
    public double tableSpinSpinPercAllocation = 0.5;

    //Distance Between swerve module and Center
    public double DISTANCE_BETWEEN_MODULE_AND_CENTER;
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;

}
