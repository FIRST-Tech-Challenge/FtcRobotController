package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
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

    final double SERVO_SLIGHTLY_OPENED_POSITION = 0.01;

    final double SERVO_SLIGHTLY_OPENED_DURATION = 0.3;

    public Claw(ServoImplEx servo, Gamepad gamepad, ElapsedTime runtime) {
        this.servo = servo;
        this.gamepad = gamepad;
        this.runtime = runtime;


    }

    private boolean aPressed = false;
    private boolean aPressedDelta = false;
    private boolean aPressedPrevious = false;

    public void update() {
        aPressed = gamepad.a;

        // if the button has been pressed and the action has not been done yet
        if (aPressedDelta) {
            // do the action
        }

        // a pressed is not changing, so set to false
        aPressedDelta = false;

        // get previous a pressed value
        aPressedPrevious = aPressed;

        // if a is pressed
        if (aPressed) {
            // if a was not pressed previously, aPressedDelta is true
            aPressedDelta = !aPressedPrevious;
        }

        if (gamepad.a) {
            servo.setPosition(SERVO_SLIGHTLY_OPENED_POSITION);
        }

//        // servo close
//        if (gamepad.a) {
//            servo.setPosition(SERVO_CLOSED_POSITION);
//            servoCloseTime = runtime.time();
//            servoSlightOpenTime = 0;
//        }
//
//        if (servoCloseTime > 0 && runtime.time() - servoCloseTime >= SERVO_CLOSED_DURATION) {
//            servo.setPosition(SERVO_SLIGHTLY_OPENED_POSITION);
//            servoSlightOpenTime = runtime.time();
//            servoCloseTime = 0;
//
//        }
//
//        if (servoSlightOpenTime > 0 && runtime.time() - servoSlightOpenTime >= SERVO_SLIGHTLY_OPENED_DURATION) {
//            servo.setPosition(SERVO_CLOSED_POSITION);
//            servoSlightOpenTime = 0;
//        }
//
//        // open
//        if (gamepad.b) {
//            servo.setPosition(SERVO_OPENED_POSITION);
//            servoCloseTime = 0;
//
//        }

    }

}
