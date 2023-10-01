package org.firstinspires.ftc.teampractice.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teampractice.examples.YellowDetector;

@TeleOp(name = "Duck Test", group = "Test")
//@Disabled
public class DuckTest extends LinearOpMode {
    double _theadshold = -1d;

    @Override
    public void runOpMode() throws InterruptedException {
        YellowDetector yellowDetector = new YellowDetector(hardwareMap);

        telemetry.addData("Analysis", new Func<String>() {
            @Override public String value() {
                return String.format("%s, %s",
                        yellowDetector.objectDetected().toString(),
                        yellowDetector.debug());
            }
        });

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.update();
            idle();
        }

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", yellowDetector.objectDetected().toString());
            telemetry.addData("Debug", yellowDetector.debug());
            telemetry.addData("Threshold", "%.2f", _theadshold);
            telemetry.update();

            if (gamepad1.dpad_up) {
                _theadshold += 1d;
            } else if (gamepad1.dpad_down) {
                _theadshold -= 1d;
            }

            if (gamepad1.a) {
                yellowDetector.pipeline.useOrignial = false;
            } else {
                yellowDetector.pipeline.useOrignial = true;
            }

            yellowDetector.pipeline.set_threshold(_theadshold);

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}
