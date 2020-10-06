package org.firstinspires.ftc.team6220_2020;

public abstract class MasterTeleOp extends MasterOpMode
{
    public void driveMecanumWithJoysticks()
    {
        double turningPower = gamepad1.left_stick_x;
        double driveAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x);
        double drivePower = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
        driveMecanum(driveAngle, drivePower, turningPower);
    }
}

