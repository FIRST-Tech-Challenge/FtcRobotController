package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo Wrist;

    public Wrist(HardwareMap hardwareMap) {
        this.Wrist = hardwareMap.get(Servo.class, "Wrist");
        this.Wrist.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {Wrist.setPosition(position);
    }
}
