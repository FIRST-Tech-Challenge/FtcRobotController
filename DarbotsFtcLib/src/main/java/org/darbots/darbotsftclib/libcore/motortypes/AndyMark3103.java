package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class AndyMark3103 implements MotorType {
    @Override
    public String getMotorName() {
        return "AndyMark3103 am-3103";
    }

    @Override
    public double getCountsPerRev() {
        return 1680;
    }

    @Override
    public double getRevPerSec() {
        return 1.75;
    }
}
