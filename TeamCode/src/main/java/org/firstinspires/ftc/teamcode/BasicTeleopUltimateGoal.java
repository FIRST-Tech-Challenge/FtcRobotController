package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * basic teleop, all aspects of the robot are manually controlled and mapped to certain buttons on the gamepad,
 *
 */


@TeleOp(name="Basic TeleOP Ultimate Goal", group="UltimateGoal")
public class BasicTeleopUltimateGoal extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareUltimateGoal robot = new HardwareUltimateGoal();


    @Override
    public void runOpMode()
    {
        // declare some variables if needed

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        //maybe some other set up stuff depending on how we want to do this
        while (opModeIsActive())
        {

        }
    }

}
