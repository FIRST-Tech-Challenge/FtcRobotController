package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/** @noinspection unused */
@Config
@TeleOp(name = "Wrist Suite", group = "TeleOp")
public class LiterallyJustTestTheWrist extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo testServo = hardwareMap.get(Servo.class, "wrist");

        waitForStart();

        // test the wrist

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                testServo.setPosition(testServo.getPosition() + 0.01);
            }
            else if (gamepad1.dpad_down) {
                testServo.setPosition(testServo.getPosition() - 0.01);
            }
            telemetry.addData("Position", testServo.getPosition());
            telemetry.addData("Target Position", "%.3f", testServo.getPosition());
            telemetry.addData("Controls", "DPAD = move");
            telemetry.update();
        }


    }
}
