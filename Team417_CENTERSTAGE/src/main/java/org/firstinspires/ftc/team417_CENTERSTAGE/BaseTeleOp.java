package org.firstinspires.ftc.team417_CENTERSTAGE;

public class BaseTeleOp extends BaseOpMode {
    @Override
    public void runOpMode() {
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            drive();
        }
    }

    public void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rotX = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotX), 1);

        double frontLeftPower = (y + x + rotX) / denominator;
        double frontRightPower = (y - x - rotX) / denominator;
        double backLeftPower = (y - x + rotX) / denominator;
        double backRightPower = (y + x - rotX) / denominator;

        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);

    }
}
