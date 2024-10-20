package com.kalipsorobotics.fresh.test;

import com.kalipsorobotics.fresh.mechanisms.LinkageServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestLinkageServo extends LinearOpMode {
    Servo linkage = hardwareMap.get(Servo.class, "linkage");
    double linkagePos = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {


            telemetry.addData("LinkageServoPos", linkage.getPosition());
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

            linkage.setPosition(linkagePos);


    }



}
