package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo claw;

    public Claw(HardwareMap hardwareMap) {
        this.claw = hardwareMap.get(Servo.class, "claw");
    }


    public void open() {
        double open = 0.74;
        claw.setPosition(open);
    }

    public void close() {
        double close = 0.14;
        claw.setPosition(close);
    }

    public void setPosition(double position) {
        claw.setPosition(position);
    }
}
