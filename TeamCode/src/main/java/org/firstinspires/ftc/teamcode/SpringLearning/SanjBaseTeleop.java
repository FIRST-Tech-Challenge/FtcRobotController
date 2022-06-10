package org.firstinspires.ftc.teamcode.SpringLearning;

abstract public class SanjBaseTeleop extends SanjBaseOpmode {
    public void driveRobotUsingControllers() {
        double yControl = - gamepad1.left_stick_y;
        double xControl = gamepad1.left_stick_x;
        double rotationalControl = gamepad1.right_stick_x;

        mecanumDrive(xControl, yControl, rotationalControl);
    }
}
