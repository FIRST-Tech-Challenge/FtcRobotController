package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp League 1")
public class BaseTeleOp extends BaseOpMode {
    @Override
    public void runOpMode() {
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            driveUsingControllers(true);
        }
    }

    public void driveUsingControllers() {
        double sensitivity = 1;
        double rotSensitivity = 1;
        double strafeConstant = 1.1;

        double x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
        double y = -curveStick(gamepad1.left_stick_y) * sensitivity;
        double rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;

        mecanumDrive(x, y, rot);
    }

    public void driveUsingControllers(boolean curve) {
        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;
        double x, y, rot;
        if (curve) {
            sensitivity = 1;
            rotSensitivity = 1;
            x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
            y = -curveStick(gamepad1.left_stick_y) * sensitivity;
            rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;
        } else {
            sensitivity = 0.5;
            rotSensitivity = 0.8;
            x = gamepad1.left_stick_x * strafeConstant * sensitivity;
            y = -gamepad1.left_stick_y * sensitivity;
            rot = gamepad1.right_stick_x * rotSensitivity;
        }
        mecanumDrive(x, y, rot);
    }

    //Adds stick curve to the drive joysticks
    public double curveStick(double rawSpeed) {
        double logSpeed;
        if (rawSpeed >= 0) {
            logSpeed = Math.pow(rawSpeed, 2);
        } else {
            logSpeed = Math.pow(rawSpeed, 2);
        }
        return logSpeed;
    }
}
