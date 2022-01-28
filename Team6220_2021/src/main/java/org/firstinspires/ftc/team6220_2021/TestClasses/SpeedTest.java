//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_2021.MasterOpMode;

@TeleOp(name = "Speed Test", group = "TeleOp")
@Disabled
public class SpeedTest extends MasterOpMode {
    double x = 0.7;

    @Override
    public void runOpMode() {
        Initialize();

        //Declare variables
        int position = 0;
        boolean isPressed = false;
        double motorPower = 0.9;
        double increase = 1;
        double oldPosition = 0;

        waitForStart();

        //Set power of motors
        while (opModeIsActive()) {
            if (gamepad1.b) {
                motorFR.setPower(gamepad1.left_stick_y);
            } else if (gamepad1.x) {
                motorFL.setPower(gamepad1.left_stick_y);
            } else if (gamepad1.a) {
                motorBR.setPower(gamepad1.left_stick_y);
            } else if (gamepad1.y) {
                motorBL.setPower(gamepad1.left_stick_y);
            }
        }
    }
}