package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Clamp extends RFServo {
    public static double CLAMP = 0.2, UNCLAMP = 0.4;
    boolean clamped;

    public Clamp() {
        super("clampServo", 1.0);
        clamped = false;
        super.setPosition(UNCLAMP);
    }

    public void clamp() {
        super.setPosition(CLAMP);
        clamped = true;
    }

    public void unclamp() {
        super.setPosition(UNCLAMP);
        clamped = false;
    }

    public boolean getClamped() {
        return clamped;
    }
}
