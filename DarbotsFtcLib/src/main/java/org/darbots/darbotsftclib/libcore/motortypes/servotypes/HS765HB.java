package org.darbots.darbotsftclib.libcore.motortypes.servotypes;

import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class HS765HB extends ServoType {
    //https://www.servocity.com/hs-765hb-servo
    @Override
    public String getServoName() {
        return "HS-765hb";
    }

    @Override
    public double getMaxDeg() {
        return 200.5;
    }

    @Override
    public double getMinDeg() {
        return 0;
    }

    @Override
    public double getPulseLowerInMicroSeconds() {
        return 680;
    }

    @Override
    public double getPulseUpperInMicroSeconds() {
        return 2220;
    }

    @Override
    public double getRevPerSec() {
        return 60.0 / 0.28 / 200.5;
        //60 deg in 0.28s => 4.8V
        //60 deg in 0.23s => 6.0V
    }
}
