/*
// simple teleop program that demonstrates timing issues.
Reference: https://stemrobotics.cs.pdx.edu/node/7262
        */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Timing Demo", group="Exercises")
//@Disabled
public class TimingDemo1 extends LinearOpMode
{
    int     buttonCount;
    boolean bButton;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.a) buttonCount += 1;

            if (gamepad1.b) bButton = !bButton;

            telemetry.addData("Mode", "running");
            telemetry.addData("A button Count", buttonCount);
            telemetry.addData("B button", bButton);
            telemetry.update();

            idle();
        }
    }
}

