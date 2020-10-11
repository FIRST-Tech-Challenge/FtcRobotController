package org.darbots.darbotsftclib.libcore.motortypes.servotypes;

import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class HS485HB extends ServoType {
    //https://www.servocity.com/hs-485hb-servo
    @Override
    public String getServoName() {
        return "HS-485hb";
    }

    @Override
    public double getMaxDeg() {
        return 190.5;
    }

    @Override
    public double getMinDeg() {
        return 0;
    }

    @Override
    public double getPulseLowerInMicroSeconds() {
        return 553;
    }

    @Override
    public double getPulseUpperInMicroSeconds() {
        return 2425;
    }

    @Override
    public double getRevPerSec() {
        return 60 / 0.22 / 190.5;
        //60 deg in 0.22s => 4.8V
        //60 deg in 0.18s => 6.0V
        //300 deg in 1s => 300 / 190 (1 rev = 90deg)
    }
}
