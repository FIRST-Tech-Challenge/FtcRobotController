package org.darbots.darbotsftclib.libcore.motortypes.servotypes;

import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

//https://www.servocity.com/hs-625mg-servo
public class HS625MG extends ServoType {
    @Override
    public String getServoName() {
        return "HS-625mg";
    }

    @Override
    public double getMaxDeg() {
        return 197;
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
        return 2520;
    }

    @Override
    public double getRevPerSec() {
        return 60 / 0.18 / 197;
        //60 deg in 0.18s => 4.8V
        //60 deg in 0.15s => 6.0V
    }
}
