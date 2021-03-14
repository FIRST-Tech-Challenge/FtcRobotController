package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team417_2020.Resources.Toggler;

abstract public class MasterTeleOp extends MasterOpMode {

    private boolean isReverseMode = false;
    private boolean isSlowMode = false;
    private boolean prevClawPosition = false;

    private Toggler slowMode = new Toggler();
    private Toggler reverseMode = new Toggler();


    /**
     * Uses the mecanum drive function to move the robot | Right stick translate, Left stick rotate (gamepad1)
     */
    public void driveRobot()
    {
        isSlowMode = slowMode.toggle(gamepad1.right_bumper);
        isReverseMode = reverseMode.toggle(gamepad1.left_bumper);

        double y = -gamepad1.right_stick_y; // Y is negative above the Y axis
        double x = gamepad1.right_stick_x;
        double rotationalPower = gamepad1.left_stick_x;

        if (isSlowMode) {
            y *= 0.2;
            x *= 0.2;
            rotationalPower *= 0.2;
        }
        else if (isReverseMode) {
            y *= -1;
            x *= -1;
        }


        // todo check and test to see if we need filtering
        /*
        filterJoyStickInput.appendInput(x, y, pivotPower);

        x = filterJoyStickInput.getFilteredX();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();
         */

        double drivePower = Math.hypot(x, y);
        double angle = Math.atan2(y, x);

        mecanumDrive(angle, drivePower, rotationalPower);
    }
}
