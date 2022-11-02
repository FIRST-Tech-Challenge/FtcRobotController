package org.firstinspires.ftc.team417_PowerPlay;

abstract public class BaseTeleOp extends BaseOpMode{
    public void driveUsingControllers() {
        double x = gamepad1.left_stick_x;
        double y = - gamepad1.left_stick_y;
        double turning = gamepad1.right_stick_x;

        mecanumDrive(x, y, turning);
    }
}
