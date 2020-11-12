package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    private Servo servo;
    public WobbleGoal(Servo servo) {
        this.servo = servo;
    }
    public void raise() {
        servo.setPosition(0);

    }

    public void lower() {
        servo.setPosition(1);

    }
}
