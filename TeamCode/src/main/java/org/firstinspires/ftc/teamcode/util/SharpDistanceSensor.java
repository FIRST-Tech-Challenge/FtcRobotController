package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

//this comes straight from Acme - test to see if it matches our sensor

public class SharpDistanceSensor {

    private AnalogInput input;
    private static final double A = .3928;
    private static final double B = -1.66868;

    public SharpDistanceSensor(AnalogInput input) {
        this.input = input;
    }

    public double getRawVoltage() {
        return input.getVoltage();
    }

    private double linearize(double v) {
        return Range.clip(A * Math.pow(v, B), .1, 5.0);
    }

    public double getUnscaledDistance() {
        return linearize(getRawVoltage());
    }
}
