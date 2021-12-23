/*
Reference: https://stemrobotics.cs.pdx.edu/node/4700
*/
package org.firstinspires.ftc.teamcode;

// Import the LinearOpMode class as well as the other classes we need.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

// Our new OpMode class extends the base LinearOpMode class.

@Autonomous(name="NullLinearOp")
public class NullLinearOp extends LinearOpMode
{

    // Start button pressed, off we go.

    /*
    private String startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date()),
            initDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
    */
    private String startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss",
                java.util.Locale.getDefault()).format(new Date()),
            initDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss",
                java.util.Locale.getDefault()).format(new Date());

    // Create an instance of the built-in ElapsedTime object from the FTC SDK.
    // It will return the current total run time of the opMode when called.

    private ElapsedTime runtime = new ElapsedTime();

    // There is only one method for us to override with our own code.

    @Override
    public void runOpMode()  throws InterruptedException
    {

        telemetry.addData("init", "NullLinearOp initialized at " + initDate);
        telemetry.update();

        // After we are done initializing our code, we wait for Start button.

        waitForStart();

        runtime.reset();  // Start counting run time from now.

        // Here we implement the loop our code needs to run in for the duration of
        // our OpModes execution. We can tell when to stop by monitoring the base
        // LinearOpMode class opModeIsActive method.

        while (opModeIsActive())
        {
            telemetry.addData("Start", "NullLinearOp started at " + startDate);
            telemetry.addData("Status", "running for " + runtime.toString());
            telemetry.update();

            idle();
        }
    }
}
