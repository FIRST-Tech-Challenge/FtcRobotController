package org.darbots.darbotsftclib.libcore.motortypes.encoders;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class USDigital_E4T_360 implements MotorType {
    @Override
    public String getMotorName() {
        return "USDigital-EST-360";
    }

    @Override
    public double getCountsPerRev() {
        return 1440;
    }

    @Override
    public double getRevPerSec() {
        return 0;
    }
}
