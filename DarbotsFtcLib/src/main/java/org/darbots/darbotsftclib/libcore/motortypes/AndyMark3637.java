package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class AndyMark3637 implements MotorType {
    @Override
    public String getMotorName() {
        return "AndyMark20Motor am-3637";
    }

    @Override
    public double getCountsPerRev() {
        return 537.6;
    }

    @Override
    public double getRevPerSec() {
        return 5.47;
        // Actually 340 RPM, but we are using NevRest 20 Motor Config, return 328.125 RPM
        // return 5.67;
    }
}
