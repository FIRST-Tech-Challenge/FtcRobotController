package com.kalipsorobotics.fresh.mechanisms;

import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class LinkageServo {

    private final Servo servo;

    public LinkageServo(Servo servo) {
        this.servo = servo;
    }

    private void moveServo(double servoPos) {
        servo.setPosition(servoPos);
    }

    public void servoExtendPos() {

    }

    public void servoRetractPos() {

    }












}
