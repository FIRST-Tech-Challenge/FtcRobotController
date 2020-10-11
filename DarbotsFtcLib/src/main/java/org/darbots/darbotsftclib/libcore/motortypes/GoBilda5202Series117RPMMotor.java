package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class GoBilda5202Series117RPMMotor implements MotorType {
    @Override
    public String getMotorName() {
        return "GoBilda 5202 Series 117 RPM Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 1425.2;
    }

    @Override
    public double getRevPerSec() {
        return 1.95;
    }
}
