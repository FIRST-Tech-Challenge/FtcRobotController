package org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ClawSubsystem extends SubsystemBase {
    private final ServoImplEx servo;
    private final double MIN = 0, MAX = 0.26;

    public enum State {
        GRAB, RELEASE
    };

    State state;

    public ClawSubsystem(HardwareMap hm) {
        this.servo = hm.get(ServoImplEx.class, "claw");
        this.release();
    }

    public void grab() {
        servo.setPosition(MAX);
        state = State.GRAB;
    }

    public void release() {
        servo.setPosition(MIN);
        state = State.RELEASE;
    }

    public void toggleState() {
        if(state == State.GRAB)
            release();
        else
            grab();
    }

    public State getState() {
        return state;
    }
}