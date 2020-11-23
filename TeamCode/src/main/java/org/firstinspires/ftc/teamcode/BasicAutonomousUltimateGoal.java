package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * basic autonomous, will probably be dead reckoning.
 *
 */
@Autonomous(name="Basic Autonomous Ultimate Goal", group="UltimateGoal")
public class BasicAutonomousUltimateGoal extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareUltimateGoal robot   = new HardwareUltimateGoal();



    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //set up other things


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        while (opModeIsActive())
        {

        }



    }
}
