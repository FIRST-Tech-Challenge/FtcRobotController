package org.firstinspires.ftc.teamcode.opModes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;

/**
 * Description: [Fill in]
 * Hardware:
 *  [motor0] Unused
 *  [motor1] Unused
 *  [motor2] Unused
 *  [motor3] Unused
 *  [servo0] Unused
 *  [servo1] Unused
 *  [servo2] Unused
 *  [servo3] Unused
 */
// @Disabled // REMEMBER TO REMOVE THIS WHEN RUNNING.
@Autonomous(name="Example Linear Auto Op Mode", group="Example")
public class ExampleLinearAutoOpMode extends AutonomousLinearModeBase {
    // Declare class members here
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void run() {
        // Code to run once INIT is pressed
        waitForStart();
        // Code to run once PLAY is pressed
        runtime.reset();
        // Run until the driver presses STOP
        while (opModeIsActive()) {
        // Code to run in a loop
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        }
    }
}