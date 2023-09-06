package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class ArmRelease {
    private static final double RELEASE_POS = 0.85;
    private static final double SET_POS = 0.2;
    private final Servo _arm_release;

    public ArmRelease(Servo arm_release) {
        this._arm_release = arm_release;
    }
    public void release() {
        _arm_release.setPosition(RELEASE_POS);
    }

    public void set() {
        _arm_release.setPosition(SET_POS);
    }
}
