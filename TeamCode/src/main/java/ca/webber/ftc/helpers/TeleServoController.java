package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleServoController {

    private Servo p_servo;

    public TeleServoController(Servo servo) {
        p_servo = servo;
    }

    public void close() {
        p_servo.setPosition(0);
    }

    public void open() {
        p_servo.setPosition(1);
    }
}
