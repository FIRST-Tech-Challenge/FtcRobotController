package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

import java.util.Random;

// Automated testing for limit switch homing
@TeleOp(name = "SlidesLimitSwitchTest")
public class SlidesLimitSwitchTest extends BaseTeleOp {
    Random rng = new Random();

    @Override
    public void runOpMode() {
        initialize();
        driveSlides(0);
        waitForStart();

        while (opModeIsActive()) {
            // Repeat moving to 5 random slide heights then homing 20 times.
            for (int i = 0; i <= 20; i++) {
                for (int j = 0; j <= 5; j++) {
                    // Drive slides to random position between ticks 50 and Constants.SLIDE_TOP
                    driveSlides(rng.nextInt(Constants.SLIDE_TOP - 50) + 50);
                }
                // Wait for a to be pressed
                while (!gamepad1.a);
                driveSlides(0);
            }

            // driveSlides() adds the encoder offset to telemetrySave so it can be printed here
            for (Object o : telemetrySave) {
                telemetry.addData("test", o);
            }
            telemetry.update();
            // Wait for a to be pressed
            while (!gamepad1.a);
        }
    }
}
