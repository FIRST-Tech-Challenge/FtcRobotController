package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.kalipsorobotics.modules.RevLED;

@TeleOp
public class RevLEDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RevLED revLED = new RevLED(hardwareMap, "redLed", "greenLed");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                RevLED.turnOnRed();
            }
            else if (gamepad1.b) {
                RevLED.turnOnGreen();
            }

        }
    }

}