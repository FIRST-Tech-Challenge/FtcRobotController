package com.kalipsorobotics.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.kalipsorobotics.modules.RevLED;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class RevLEDTest extends LinearOpMode {
    private RevLED revLED;

    @Override
    public void runOpMode() throws InterruptedException {
        revLED = new RevLED(hardwareMap,"red", "green", "red2","green2","red3","green3","red4","green4");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                revLED.turnOnRed();
            } else if (gamepad1.b) {
                revLED.turnOnGreen();
            } else {
                revLED.turnoff();
            }

        }
    }
}
