/*
// simple autonomous program that drives bot forward 2 seconds then ends.
Reference: https://stemrobotics.cs.pdx.edu/node/4719
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name="Drive Forward", group="Exercises")
//@Disabled
public class DriveForward extends LinearOpMode
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

        // set both motors to 25% power.

        leftMotor.setPower(0.25);
        rightMotor.setPower(0.25);

        sleep(2000);        // wait for 2 seconds.

        // set motor power to zero to stop motors.

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}
