package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class AndyMark3102 implements MotorType {
    @Override
    public String getMotorName() {
        return "AndyMark3102 am-3102";
    }

    @Override
    public double getCountsPerRev() {
        return 560;
    }

    @Override
    public double getRevPerSec() {
        return 4.58;
    }
}
