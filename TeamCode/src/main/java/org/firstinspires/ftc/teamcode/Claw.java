package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    private ServoImplEx servo;
    private Gamepad gamepad;
    private ElapsedTime runtime;

    private double servoCloseTime;
    private double servoSlightOpenTime;

    final double SERVO_OPENED_POSITION = 0;

    final double SERVO_CLOSED_POSITION = 1;

    final double SERVO_CLOSED_DURATION = 1;

    final double SERVO_SLIGHTLY_OPENED_POSITION = 0.5;

    final double SERVO_SLIGHTLY_OPENED_DURATION = 0.3;

    private boolean isPickingUp = false;

    public Claw(ServoImplEx servo, Gamepad gamepad, ElapsedTime runtime) {
        // pulse width modulation??
        servo.setPwmEnable();
        servo.setPwmRange(new PwmControl.PwmRange(1400, 1900));
        this.servo = servo;
        this.gamepad = gamepad;
        this.runtime = runtime;
    }

    public void update() {

        // servo close
        if (gamepad.a && !isPickingUp) {
            servo.setPosition(SERVO_CLOSED_POSITION);
            servoCloseTime = runtime.time();
            servoSlightOpenTime = 0;
            isPickingUp = true;
        }

        if (isPickingUp) {
            if (servoCloseTime > 0 && runtime.time() - servoCloseTime >= SERVO_CLOSED_DURATION) {
                servo.setPosition(SERVO_SLIGHTLY_OPENED_POSITION);
                servoSlightOpenTime = runtime.time();
                servoCloseTime = 0;

            }

            if (servoSlightOpenTime > 0 && runtime.time() - servoSlightOpenTime >= SERVO_SLIGHTLY_OPENED_DURATION) {
                servo.setPosition(SERVO_CLOSED_POSITION);
                servoSlightOpenTime = 0;
                isPickingUp = false;
            }
        }


        // open
        if (gamepad.b) {
            servo.setPosition(SERVO_OPENED_POSITION);
            servoCloseTime = 0;
            isPickingUp = false;
        }

    }

}
