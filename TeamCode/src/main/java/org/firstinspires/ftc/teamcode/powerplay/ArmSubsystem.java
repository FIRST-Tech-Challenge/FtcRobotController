package org.firstinspires.ftc.teamcode.powerplay;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private final Servo servo;

    enum State {
        BACKWARD,
        MIDDLE,
        FORWARD
    }

    State state;

    public void setForward() {
        servo.setPosition(0.2);
        state = State.FORWARD;
    }

    public void setBackward() {
        servo.setPosition(0.8);
        state = State.BACKWARD;
    }

    public void setMiddle() {
        servo.setPosition(0.5);
        state = State.MIDDLE;
    }

    public void toggleState() {
        if (state == State.FORWARD)
            setBackward();
        else
            setForward();
    }

    public ArmSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "arm");
    }
}
