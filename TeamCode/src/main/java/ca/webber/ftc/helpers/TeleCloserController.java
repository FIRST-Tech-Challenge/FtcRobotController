package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.Servo;

public class TeleCloserController {

    private Servo p_servo;
    private static final double INCREMENT = 0.01;
    private boolean lastState = false;

    public TeleCloserController (Servo servo) {
        p_servo = servo;
        p_servo.setPosition(0);
    }

    public void update (boolean isPressed) {
        if (isPressed && !lastState) {
            toggle();
        }

        lastState = isPressed;
    }

    public void toggle () {
        p_servo.setPosition(1 - p_servo.getPosition());
    }
}
