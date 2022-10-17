package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {
    private Button toggle;
    private Servo servo;
    private double position;

    public Bucket(HardwareMap hardwareMap, ExtendedGamepad gamepad2) {
        this.toggle = gamepad2.a;
        this.servo = hardwareMap.get(Servo.class, "bucket");
        this.servo.setPosition(0.87);
        this.position = 1;
    }

    public void run() {
        if (toggle.isBumped()) {
            if (position == 0) {
                servo.setPosition(0.87);
                position = 1;
            } else {
                servo.setPosition(0.18);
                position = 0;
            }
        }
    }

    public void setVertical() {
        servo.setPosition(0.75);
        position = 0.5;
    }

    public void setIntake() {
        servo.setPosition(0.87);
        position = 1;
    }

    public void setRelease() {
        servo.setPosition(0.18);
        position = 0;
    }
}