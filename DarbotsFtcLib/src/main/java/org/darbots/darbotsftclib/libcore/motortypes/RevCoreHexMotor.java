package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class RevCoreHexMotor implements MotorType {
    @Override
    public String getMotorName() {
        return "Rev Core Hex Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 288;
    }

    @Override
    public double getRevPerSec() {
        return 2.08;
    }
}
