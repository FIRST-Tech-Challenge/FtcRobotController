package org.firstinspires.ftc.teamcode.powerplay;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    private final Servo servo;

    public enum State {
        REST, INTAKE, RELEASE
    };

    State state;

    public ClawSubsystem(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(Servo.class, "bucket");
        this.release();
    }

    public void grab() {
        servo.setPosition(0.75);
        state = State.REST;
    }

    public void release() {
        servo.setPosition(0.18);
        state = State.RELEASE;
    }

    public State getState() {
        return state;
    }
}