package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestLinkageServo extends LinearOpMode {
    Servo linkageServo1;
    Servo linkageServo2;
    double linkagePos = 0.005;


    @Override
    public void runOpMode() throws InterruptedException {
        linkageServo1 = hardwareMap.get(Servo.class, "linkageServo1");
        linkageServo2 = hardwareMap.get(Servo.class, "linkageServo2");
        linkageServo1.setPosition(linkagePos);
        linkageServo2.setPosition(linkagePos);
        linkageServo1.setDirection(Servo.Direction.REVERSE);

        double Speed = 0.05;

        waitForStart();

        while (opModeIsActive()) {
            double servoPos = linkageServo1.getPosition();
            double servoPos2 = linkageServo2.getPosition();

            if (servoPos != servoPos2) {
                telemetry.addData("Positions are not aligned, Servo 1:", servoPos);
                telemetry.addData("Servo 2", servoPos2);
            }
            telemetry.addData("Linkage Pos: ", servoPos);
            telemetry.update();

            if (gamepad1.right_stick_y < 0) {
                linkagePos += Speed;
            } else if (gamepad1.right_stick_y > 0) {
                linkagePos -= Speed;
            }

            linkagePos = Math.max(0, Math.min(1, linkagePos));

            if (gamepad1.dpad_up) {
                Speed = Math.max(0.1, Speed - 0.05);
            } else if (gamepad1.dpad_down) {
                Speed = Math.min(1, Speed + 0.05);
            }



            linkageServo1.setPosition(linkagePos);
            linkageServo2.setPosition(linkagePos);
        }
    }
}
