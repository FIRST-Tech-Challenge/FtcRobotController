package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private final Servo wrist;
    public double lastPosition = 0;


    public Wrist(HardwareMap hardwareMap) {
        this.wrist = hardwareMap.get(Servo.class, "wrist");
    }

    public void setPosition(double position) {
        if (position != lastPosition) {
            wrist.setPosition(position);
        }
        lastPosition = position;
    }

    public void intake() {
        setPosition(0.48);
    }

    public void deposit() {
        setPosition(.95);
    }

    public void holdSpecimen() {
        setPosition(1);
    }
    public void specimen() {
        setPosition(.55);
    }


}
