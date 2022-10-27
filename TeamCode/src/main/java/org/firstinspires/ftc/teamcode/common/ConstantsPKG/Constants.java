package org.firstinspires.ftc.teamcode.common.ConstantsPKG;

import org.checkerframework.checker.units.qual.C;

public class Constants {
    public double LOOP_ITERATION_TIME = 0.025; //must test later on

    //Drive Train Constants
    public double LOAD_ON = 0.6; //assumption
    public double RPM = 1150 * LOAD_ON; //690.  Not very accurate so don't rely on this number.
    public double RPS = RPM / 60.0; //11.5 ish motor revolutions per second, with load

    public double CLICKS_PER_BLUE_REV = 145.1; //clicks per rev of motor

    public double WHEEL_DIAMETER = 92 / 25.4; //unit is in inches
    public double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public double MAX_VELOCITY_DT = 2700; // unit is clicks/sec
    public double clickTOLERANCE = 3; //number of clicks or degrees the robot can be off by
    public double degreeTOLERANCE = 1;

    //Swerve constants
        //module translation
    public double BLUE_REV_PER_GREEN = 17.0 / 16.0; //assuming other gear isn't messing stuff up, 17 revolutions of the input shaft = 16 revolutions of the wheel
    public double CLICKS_PER_INCH = BLUE_REV_PER_GREEN * CLICKS_PER_BLUE_REV * (1.0 / WHEEL_CIRCUMFERENCE);
    public double INCHES_PER_CLICK = 1.0 / CLICKS_PER_INCH; //~0.029 inches per click

    //module rotation
    public double BLUE_REVS_PER_PURPLE = 85.0 / 24.0; //~3.54 Blue revs per 1 Purple rev
    public double CLICKS_PER_DEGREE = BLUE_REVS_PER_PURPLE * CLICKS_PER_BLUE_REV * (1/360.0);
    public double DEGREES_PER_CLICK = 1.0 / CLICKS_PER_DEGREE;

    public double DEGREES_PER_INCH = CLICKS_PER_INCH * DEGREES_PER_CLICK;
    public double INCHES_PER_DEGREE = 1.0 / DEGREES_PER_INCH;

    public double tableSpinRotPercAllocation = 0.5;
    public double tableSpinSpinPercAllocation = 0.5;

    //Distance Between swerve module and Center
    public double DISTANCE_BETWEEN_MODULE_AND_CENTER = 3.406; //3.405512
    public double horizontalDistanceOdo = 6.25;
    public double midDistanceOdo = 3.0;
}
