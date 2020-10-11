package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class AndyMark2964 implements MotorType {

    @Override
    public String getMotorName() {
        return "AndyMark2964 am-2964";
    }

    @Override
    public double getCountsPerRev() {
        return 1120;
    }

    @Override
    public double getRevPerSec() {
        return 2.67;
    }
}
