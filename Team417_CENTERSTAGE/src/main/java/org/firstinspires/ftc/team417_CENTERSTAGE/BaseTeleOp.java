package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp League 1")
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
        double sensitivity = 0.5;
        double rotSensitivity = 0.8;
        double strafeConstant = 1.1;

        double y = -gamepad1.left_stick_y * sensitivity;
        double x = gamepad1.left_stick_x * strafeConstant * sensitivity;
        double rotX = gamepad1.right_stick_x * rotSensitivity;

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
