/*
// simple teleop program that drives bot using controller joysticks in arcade mode.
// this code monitors the period and stops when the period is ended.
Reference: https://stemrobotics.cs.pdx.edu/node/4737
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive Arcade", group="Exercises")
//@Disabled
public class DriveArcade extends LinearOpMode
{
    DcMotor leftMotor, rightMotor;
    float   leftPower, rightPower, xValue, yValue;

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

        while (opModeIsActive())
        {
            yValue = gamepad1.right_stick_y * -1;
            xValue = gamepad1.right_stick_x * -1;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            leftMotor.setPower(Range.clip(leftPower, -1.0, 1.0));
            rightMotor.setPower(Range.clip(rightPower, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.update();

            idle();
        }
    }
}
