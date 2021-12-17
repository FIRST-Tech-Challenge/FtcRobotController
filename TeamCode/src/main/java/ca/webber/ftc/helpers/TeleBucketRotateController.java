package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.Servo;

public class TeleBucketRotateController {

    final private double INCREMENT = 0.01;
    private Servo p_servo;
    private double p_angle;

    public TeleBucketRotateController(Servo servo) {
        p_servo = servo;
        p_angle = 0;
    }

    public void increase() {
        p_servo.setPosition(p_angle +=  INCREMENT);
    }

    public void decrease() {
        p_servo.setPosition(p_angle -=  INCREMENT);
    }
}
