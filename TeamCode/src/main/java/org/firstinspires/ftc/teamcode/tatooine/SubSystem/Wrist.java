package org.firstinspires.ftc.teamcode.tatooine.SubSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private final double OPEN = 1;
    private final double CLOSE = 0;
    private double currentPos = 0;
    private Servo wrist = null;

    public Wrist(HardwareMap hardwareMap) {
        this.wrist = hardwareMap.get(Servo.class, "wrist");
    }

    public void samples() {
        wrist.setPosition(OPEN);
    }

    public void specimen() {
        wrist.setPosition(CLOSE);
    }

    public void changeState() {
        if (currentPos == OPEN) {
            wrist.setPosition(CLOSE);
            currentPos = CLOSE;
        } else {
            wrist.setPosition(OPEN);
            currentPos = OPEN;
        }
    }

    public double getCurrentPos() {
        return currentPos;
    }

    public void setCurrentPos(double currentPos) {
        this.currentPos = currentPos;
    }

    public Servo getWrist() {
        return wrist;
    }

    public void setWrist(Servo wrist) {
        this.wrist = wrist;
    }
}
