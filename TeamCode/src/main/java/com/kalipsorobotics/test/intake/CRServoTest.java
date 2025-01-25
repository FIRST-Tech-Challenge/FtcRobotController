package com.kalipsorobotics.test.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp
public class CRServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo crServo = hardwareMap.get(CRServo.class, "crServo");
        crServo.setPower(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_stick_y != 0) {
                crServo.setPower(-gamepad1.right_stick_y);
            }
            telemetry.addData("Joystick Pos:", gamepad1.right_stick_y);
            telemetry.addData("Servo Pos", crServo.getPower());
            telemetry.update();
        }
    }
}
