package org.darbots.darbotsftclib.libcore.motortypes.servotypes;

import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

//https://www.servocity.com/hs-625mg-servo
public class HS785HB extends ServoType {
    @Override
    public String getServoName() {
        return "HS-785hb";
    }

    @Override
    public double getMaxDeg() {
        return 2826;
    }

    @Override
    public double getMinDeg() {
        return 0;
    }

    @Override
    public double getPulseLowerInMicroSeconds() {
        return 600;
    }

    @Override
    public double getPulseUpperInMicroSeconds() {
        return 2400;
    }

    @Override
    public double getRevPerSec() {
        return 60 / 0.18 / 2826;
        //360 deg in 0.168s => 4.8V
        //360 deg in 0.14s => 6.0V
    }
}
