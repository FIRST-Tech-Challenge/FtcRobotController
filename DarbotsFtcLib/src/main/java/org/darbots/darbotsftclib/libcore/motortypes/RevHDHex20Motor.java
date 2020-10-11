package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class RevHDHex20Motor implements MotorType {

    @Override
    public String getMotorName() {
        return "Rev HD Hex 20 Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 560;
    }

    @Override
    public double getRevPerSec() {
        return 5;
    }
}
