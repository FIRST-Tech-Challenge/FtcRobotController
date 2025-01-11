package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo claw;

    public Claw(HardwareMap hardwareMap) {
        this.claw = hardwareMap.get(Servo.class, "claw");
    }

    double open = 0.8;
    double close = 0.5;
    public void open() {
        claw.setPosition(open);
    }

    public void close() {
        claw.setPosition(close);
    }

    public void setPosition(double position) {
        claw.setPosition(position);
    }
}
