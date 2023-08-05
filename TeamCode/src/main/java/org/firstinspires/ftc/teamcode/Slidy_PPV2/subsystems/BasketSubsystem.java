package org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class BasketSubsystem extends SubsystemBase {
    private ServoImplEx servo;
    private final double MIN = 0.01, MAX = 0.48;

    public enum State {
        OUTTAKE,
        TRAVEL
    }

    State state;

    public BasketSubsystem(HardwareMap hm) {
        servo = hm.get(ServoImplEx.class, "basket");
        state = State.TRAVEL;
    }

    public void toggleState() {
        if (state == State.OUTTAKE)
            setTravel();
        else
            setOuttake();
    }

    public void setTravel() {
        servo.setPosition(MIN);
        state = State.TRAVEL;
    }

    public void setOuttake() {
        servo.setPosition(MAX);
        state = State.OUTTAKE;
    }

    public State getState() {
        return this.state;
    }
}
