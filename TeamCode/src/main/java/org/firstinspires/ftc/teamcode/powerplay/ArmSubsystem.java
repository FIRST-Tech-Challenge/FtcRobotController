package org.firstinspires.ftc.teamcode.powerplay;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private final Servo servo;

    public enum State {
        FORWARD, REST, BACKWARD
    };

    State state;

    public ArmSubsystem(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(Servo.class, "arm");
        this.rest();
    }

    //TODO Fix Servo Values
    public void forward() {
        servo.setPosition(0.87);
        state = State.FORWARD;
    }

    public void rest() {
        servo.setPosition(0.75);
        state = State.REST;
    }

    public void backward() {
        servo.setPosition(0.18);
        state = State.BACKWARD;
    }

    public State getState() {
        return state;
    }
}