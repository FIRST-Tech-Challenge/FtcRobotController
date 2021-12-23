/*
// simple teleop program that drives bot using controller joysticks in tank mode.
// this code monitors the period and stops when the period is ended.
Reference: https://stemrobotics.cs.pdx.edu/node/4740
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive Tank", group="Exercises")
//@Disabled
public class DriveTank extends LinearOpMode
{
    DcMotor leftMotor, rightMotor;
    float   leftY, rightY;

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
            leftY = gamepad1.left_stick_y * -1;
            rightY = gamepad1.right_stick_y * -1;

            leftMotor.setPower(Range.clip(leftY, -1.0, 1.0));
            rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);
            telemetry.update();

            idle();
        }
    }
}
