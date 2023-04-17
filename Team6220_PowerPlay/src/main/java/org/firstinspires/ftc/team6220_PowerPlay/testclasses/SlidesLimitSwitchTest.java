/*package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

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
        driveSlidesLoop(0);
        telemetry.addData("DONE", "DONE");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Repeat moving to 5 random slide heights then homing 20 times.
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    // Drive slides to random position between ticks 50 and Constants.SLIDE_TOP
                    int targetPosition = rng.nextInt(Constants.SLIDE_TOP - 200) + 50;
                    telemetry.addData("homing count", i+1);
                    telemetry.addData("random position count", j+1);
                    telemetry.addData("target position", targetPosition);
                    telemetry.addData("encoder Position", motorLeftSlides.getCurrentPosition());
                    telemetry.update();
                    driveSlidesLoop(targetPosition);
                }

                sleep(500);
                telemetry.addLine("homing");
                telemetry.update();
                driveSlidesLoop(0);
                telemetry.addLine("done");
                telemetry.update();
                sleep(1000);
            }

            // driveSlides() adds the encoder offset to telemetrySave so it can be printed here
            for (Object o : telemetrySave) {
                telemetry.addData("test", o);
            }
            telemetry.update();
            sleep(100000);
        }
    }
}
*/