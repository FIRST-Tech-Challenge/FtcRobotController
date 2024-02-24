package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class DroneSubsystem extends SubsystemBase {
    private ServoImplEx servo;

    private final double SECURE_POS = 0, RELEASE_POS = 0.25;

    public enum State {
        SECURED,
        RELEASED
    }

    private State state;

    public DroneSubsystem(HardwareMap hm) {
        servo = hm.get(ServoImplEx.class, "drone");
    }

    public void grab() {
        state = State.SECURED;
        servo.setPosition(SECURE_POS);
    }

    public void release() {
        state = State.RELEASED;
        servo.setPosition(RELEASE_POS);
    }

    public void linearMove(double value) {
        servo.setPosition(value * (RELEASE_POS-SECURE_POS) + SECURE_POS);
    }

    public State getState() {
        return state;
    }
}
