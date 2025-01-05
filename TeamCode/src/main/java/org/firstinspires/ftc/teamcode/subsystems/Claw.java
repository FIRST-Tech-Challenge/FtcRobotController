package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    private Servo claw;

    public Claw(HardwareMap hmap) {
        this.claw = hmap.get(Servo.class, "claw");
    }

    public void openClaw() {
        this.claw.setPosition(0);
    }

    public void closeClaw() {
        this.claw.setPosition(1);
    }
}
