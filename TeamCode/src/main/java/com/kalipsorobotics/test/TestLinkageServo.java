package com.kalipsorobotics.test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestLinkageServo extends LinearOpMode {
    Servo linkageServo = hardwareMap.get(Servo.class, "linkageServo");
    double linkagePos = 0.5;
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {


            telemetry.addData("LinkageServoPos", linkageServo.getPosition());
            telemetry.update();
            if (gamepad1.right_stick_y < 0) {
                linkagePos += 0.0095;
            } else if (gamepad1.right_stick_y > 0) {
                linkagePos -= 0.0095;
            }

            if (linkagePos<0) {
                linkagePos = 0;
            } else if (linkagePos>1);
                linkagePos = 1;
            }

            linkageServo.setPosition(linkagePos);


    }



}
