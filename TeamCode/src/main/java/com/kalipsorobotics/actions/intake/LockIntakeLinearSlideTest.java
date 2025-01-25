package com.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LockIntakeLinearSlideTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo ratchetServo = hardwareMap.get(Servo.class, "ratchetServo");
        double pos = 0;
        double hoverPos = 0.262;
        ratchetServo.setPosition(hoverPos);

        waitForStart();
        while (opModeIsActive()) {
//            if (gamepad1.dpad_up && pos < 1.0) {
//                pos += 0.001;
//                ratchetServo.setPosition(pos);
//
//            }
//            else if (gamepad1.dpad_down && pos > 0.0){
//                pos -= 0.001;
//                ratchetServo.setPosition(pos);
//
//            }
            if (gamepad1.a) {
                ratchetServo.setPosition(0.1);
            }

            else if (gamepad1.b) {
                ratchetServo.setPosition(hoverPos);
            }
            telemetry.addLine(String.valueOf(pos));
            telemetry.update();
        }
    }
}
// 0.128