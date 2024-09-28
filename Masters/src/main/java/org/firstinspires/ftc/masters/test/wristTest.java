package org.firstinspires.ftc.masters.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.old.CSCons;

@TeleOp(name = "Wrist Test")
public class wristTest extends LinearOpMode {

    private Servo wristServo;

    public void runOpMode() throws InterruptedException {

        wristServo = hardwareMap.servo.get("wrist");
        wristServo.setPosition(CSCons.wristVertical);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                wristServo.setPosition(0);
            }

            if (gamepad1.b) {
                wristServo.setPosition(1);
            }

        }
    }
}

