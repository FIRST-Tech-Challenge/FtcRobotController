package org.firstinspires.ftc.teamcode.powerplayV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClawSubsystem extends SubsystemBase {
    private final ServoImplEx servo;
    private final double MIN = 0.08, MAX = 0.29;

    public enum State {
        GRAB, RELEASE
    };

    State state;

    public ClawSubsystem(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(ServoImplEx.class, "claw");
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