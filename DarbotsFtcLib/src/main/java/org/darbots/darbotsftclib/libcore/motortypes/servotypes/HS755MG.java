package org.darbots.darbotsftclib.libcore.motortypes.servotypes;

import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class HS755MG extends ServoType {
    //https://www.servocity.com/hs-755mg-servo
    @Override
    public String getServoName() {
        return "HS-755mg";
    }

    @Override
    public double getMaxDeg() {
        return 200.5;
        //Actually 200.5
    }

    @Override
    public double getMinDeg() {
        return 0;
    }

    @Override
    public double getPulseLowerInMicroSeconds() {
        return 570;
    }

    @Override
    public double getPulseUpperInMicroSeconds() {
        return 2400;
    }

    @Override
    public double getRevPerSec() {
        return 60.0 / 0.28 / 200.5;
        //60 deg in 0.28s => 4.8V
        //60 deg in 0.23s => 6.0V
    }
}
