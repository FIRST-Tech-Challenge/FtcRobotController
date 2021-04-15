package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.DriveTrain;

/**
 * This <b>should</b> make a 12V DC Motor spin
 * Name of DCMotor needs to be <code>spin</code>
 * @author Joshua Miller <22jmiller@xbhs.net>
 * @version 1.0.0
 */

@TeleOp
public class DriveTrainTesting extends LinearOpMode {
    // Lets you get the Elapsed time of the program (big shocker)
    private ElapsedTime runtime = new ElapsedTime();

    DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        {
            // Not too sure why we have this twice, but I currently do not have the controller with me and can not test the usefulness of this
            telemetry.addData("Status", "Initialized");

            driveTrain = new DriveTrain(hardwareMap,gamepad1, DriveTrain.DRIVE_MODE_MIDDLE_PIVOT);

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

        // Waiting for the start button
        {
            waitForStart(); // Waits for start, somewhere there is a play button to press
        }

        // Prep before going into loop
        {
            runtime.reset(); // Resets the runtime to when the program actually runs

        }

        // Code while running
        while (opModeIsActive()) { // Only runs until the stop button is pressed
            driveTrain.update();
            // Tell about the current state
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "foo");
            telemetry.update(); // Update the screen
        }
    }
}
