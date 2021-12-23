/*
// simple teleop program that demonstrates timing issues.
Reference: https://stemrobotics.cs.pdx.edu/node/7262
        */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Timing Demo 2", group="Exercises")
//@Disabled
public class TimingDemo2 extends LinearOpMode
{
    int     buttonCount;
    boolean bButton, aButtonPressed, bButtonPressed;

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
            if (gamepad1.a)
                if(!aButtonPressed)
                {
                    buttonCount += 1;
                    aButtonPressed = true;
                }
                // reserved for future use
                // else {}
            else
                aButtonPressed = false;

            if (gamepad1.b)
                if(!bButtonPressed)
                {
                    bButton = !bButton;
                    bButtonPressed = true;
                }
                // reserved for future use
                // else {}
            else
                bButtonPressed = false;

            telemetry.addData("Mode", "running");
            telemetry.addData("A button Count", buttonCount);
            telemetry.addData("B button", bButton);
            telemetry.update();

            idle();
        }
    }
}
