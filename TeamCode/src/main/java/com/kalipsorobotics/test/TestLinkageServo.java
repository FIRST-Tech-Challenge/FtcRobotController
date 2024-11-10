package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestLinkageServo extends LinearOpMode {
    Servo linkageServo = hardwareMap.get(Servo.class, "linkageServo");
    double linkagePos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        double Speed = 0.5;
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("LinkageServoPos", linkageServo.getPosition());
            telemetry.update();
            if (gamepad1.right_stick_y < 0) {
                linkagePos += Speed;
            } else if (gamepad1.right_stick_y > 0) {
                linkagePos -= Speed;
            }

            if (linkagePos < 0) {
                linkagePos = 0;
            } else if (linkagePos > 1) {
                linkagePos = 1;
            }

            if (gamepad1.dpad_up) {
                Speed = Math.max(0.1, Speed - 0.05);
            } else if (gamepad1.dpad_down) {
                Speed = Math.min(1, Speed + 0.05);
            }

            linkageServo.setPosition(linkagePos);
        }
    }
}