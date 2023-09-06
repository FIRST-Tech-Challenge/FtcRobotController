package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ArmRelease implements SubSystem {
    private final Servo _arm_release;

    public ArmRelease(Servo arm_release) {
        this._arm_release = arm_release;
    }
    public void release() {
        _arm_release.setPosition(Constants.ARM_RELEASE_POS);
    }

    public void set() {
        _arm_release.setPosition(Constants.ARM_SET_POS);
    }
}
