package org.firstinspires.ftc.team417_2021;

abstract public class MasterTeleOp extends MasterOpMode {

    public void driveRobotUsingController() {
        double drivePower = - gamepad1.right_stick_y;

        double rotationalPower = gamepad1.left_stick_x;

        drive(drivePower, rotationalPower);
    }

}
