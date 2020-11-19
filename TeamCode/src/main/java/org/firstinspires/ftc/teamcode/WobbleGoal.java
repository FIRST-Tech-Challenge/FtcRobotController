package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    private Servo armServo;
    private Servo handServo;
    public WobbleGoal(Servo armServo, Servo handServo) {
        this.armServo = armServo;
        this.handServo = handServo;
    }
    public void raise() {
        armServo.setPosition(0);

    }

    public void lower() {
        armServo.setPosition(1);

    }
    public void open() {
        handServo.setPosition(0);
    }
    public void close() {
        handServo.setPosition(1);
    }
}
