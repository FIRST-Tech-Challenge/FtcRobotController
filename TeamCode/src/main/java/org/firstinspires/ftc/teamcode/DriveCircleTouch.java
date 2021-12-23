/*
// simple autonomous program that drives bot in a circle then ends.
// this code assumes it will end before the period is over but if the period ended while
// still driving, this code will just stop. Stops after 5 seconds or on touch sensor button.
Reference: https://stemrobotics.cs.pdx.edu/node/4744
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="Drive Circle Touch", group="Exercises")
//@Disabled
public class DriveCircleTouch extends LinearOpMode
{
    DcMotor     leftMotor;
    DcMotor     rightMotor;
    TouchSensor touch;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        touch = hardwareMap.touchSensor.get("touch_sensor");

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // set power levels 75% left and 10% right to drive in an arc to the right.

        leftMotor.setPower(0.75);
        rightMotor.setPower(0.20);

        resetStartTime();
        // drive until touch sensor button pressed or 5 seconds passes.

        while (getRuntime() < 5 && !touch.isPressed()) { idle(); }

        // turn the motors off.

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
