package org.firstinspires.ftc.team417_2021;

abstract public class MasterTeleOp extends MasterOpMode {

    public void driveRobotUsingController() {
        double drivePower = - gamepad1.right_stick_y;

        double rotationalPower = gamepad1.left_stick_x;

        drivePower *= 1 - (0.8 * gamepad1.right_trigger);
        rotationalPower *= 1 - (0.8 * gamepad1.right_trigger);
        /*telemetry.addData("FL", motorFL.getCurrentPosition());
        telemetry.addData("FR", motorFR.getCurrentPosition());
        telemetry.addData("BL", motorBL.getCurrentPosition());
        telemetry.addData("BR", motorBR.getCurrentPosition());
        telemetry.update();
*/
        drive(drivePower, rotationalPower);
    }

}
