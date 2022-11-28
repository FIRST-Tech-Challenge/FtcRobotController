package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BucketSubsystem extends SubsystemBase {
    private final Servo servo;

    public enum State {
        REST, INTAKE, RELEASE
    };

    State state;

    public BucketSubsystem(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(Servo.class, "bucket");
        this.intake();
    }

    public void rest() {
        servo.setPosition(0.75);
        state = State.REST;
    }

    public void intake() {
        servo.setPosition(0.87);
        state = State.INTAKE;
    }

    public void release() {
        servo.setPosition(0.18);
        state = State.RELEASE;
    }

    public State getState() {
        return state;
    }
}