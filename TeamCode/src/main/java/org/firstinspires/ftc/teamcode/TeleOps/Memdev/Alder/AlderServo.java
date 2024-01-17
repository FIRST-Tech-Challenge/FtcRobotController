package org.firstinspires.ftc.teamcode.TeleOps.Memdev.Alder;

import com.qualcomm.robotcore.hardware.Servo;

public class AlderServo {

    private Servo servo;

    public AlderServo(Servo s) {
        servo = s;
    }

    public void run(double power) {
        servo.setPosition(power);
    }

}
