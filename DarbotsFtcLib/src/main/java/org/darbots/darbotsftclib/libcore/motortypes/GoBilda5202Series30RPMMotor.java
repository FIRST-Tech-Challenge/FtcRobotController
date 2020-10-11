package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class GoBilda5202Series30RPMMotor implements MotorType {
    @Override
    public String getMotorName() {
        return "GoBlida 5202 Series 30RPM Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 1316;
    }

    @Override
    public double getRevPerSec() {
        return 0.5;
    }
}
