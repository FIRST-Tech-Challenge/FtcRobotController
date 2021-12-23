/*
// simple autonomous program that drives bot in a circle then ends.
// this code assumes it will end before the period is over but if the period ended while
// still driving, this code will just stop.
Reference: https://stemrobotics.cs.pdx.edu/node/4722
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Drive Circle", group="Exercises")
//@Disabled
public class DriveCircle extends LinearOpMode
{
    DcMotor leftMotor;
    DcMotor rightMotor;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(500);              // wait so that above telemetry is visible.

        // set power levels 75% left and 10% right to drive in an arc to the right.

        leftMotor.setPower(0.75);
        rightMotor.setPower(0.20);

        sleep(5000);            // drive 5 seconds to make a circle.

        // turn the motors off.

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}

