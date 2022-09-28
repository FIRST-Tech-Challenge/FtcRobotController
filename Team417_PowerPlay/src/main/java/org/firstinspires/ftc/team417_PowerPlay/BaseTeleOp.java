package org.firstinspires.ftc.team417_PowerPlay;

abstract public class BaseTeleOp extends BaseOpMode{
    public void driveUsingControllers() {
        double Xdrive = gamepad1.left_stick_x;
        double Ydrive = - gamepad1.left_stick_y;
        double Turningdrive = gamepad1.right_stick_x;

        mecanumDrive(Xdrive, Ydrive, Turningdrive);
    }
}
