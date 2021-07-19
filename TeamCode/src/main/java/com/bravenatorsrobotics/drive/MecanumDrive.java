package com.bravenatorsrobotics.drive;

import com.bravenatorsrobotics.core.Robot;

public class MecanumDrive extends FourWheelDrive {

    public MecanumDrive(Robot<? extends MecanumDrive> robot) {
        super(robot);
    }

    protected static double ScalePower(double value, double max) {
        if(max == 0) { return 0; }
        return value / max;
    }

    @Override
    public void Drive(double v, double h, double r) {
        // Calculate Motors Speeds
        double frontLeft    = v - h + r;
        double frontRight   = v + h - r;
        double backRight    = v - h - r;
        double backLeft     = v + h + r;

        // Limit the vectors to under 1
        double max = Math.max(
                Math.abs(backLeft),
                Math.max(
                        Math.abs(backRight),
                        Math.max(
                                Math.abs(frontLeft), Math.abs(frontRight)
                        )
                )
        );

        // Scale the power
        if(max > 1) { // Only scale if max is greater than one
            frontLeft   = ScalePower(frontLeft, max);
            frontRight  = ScalePower(frontRight, max);
            backLeft    = ScalePower(backLeft, max);
            backRight   = ScalePower(backRight, max);
        }

        SetPower(this.frontLeft, frontLeft);
        SetPower(this.frontRight, frontRight);
        SetPower(this.backLeft, backLeft);
        SetPower(this.backRight, backRight);
    }
}
