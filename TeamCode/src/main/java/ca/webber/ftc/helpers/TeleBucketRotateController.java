package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.Servo;

public class TeleBucketRotateController {

    private static final double p_INCREMENT = 0.01;
    private Servo p_servo;

    public TeleBucketRotateController(Servo servo) {
        p_servo = servo;
    }

    public void increase() {
        p_servo.setPosition(p_servo.getPosition() + p_INCREMENT);
    }

    public void decrease() {
        p_servo.setPosition(p_servo.getPosition() - p_INCREMENT);
    }
}
