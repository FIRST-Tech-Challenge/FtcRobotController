package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;


@TeleOp(name = "Red Drive Program")
public class RedDriveProgram extends TeleopTemplate {
    private static final RevBlinkinLedDriver.BlinkinPattern defaultColor = RevBlinkinLedDriver.BlinkinPattern.RED;
    public void runOpMode() throws InterruptedException {

        this.initAll();
        leds.setPattern(defaultColor);
        waitForStart();
        RevBlinkinLedDriver.BlinkinPattern currentColor = defaultColor;

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            //Handles Linear Slide Control
            slide.setMotorPower(1 * gamepad2.left_stick_y);
            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
                RevBlinkinLedDriver.BlinkinPattern o = intake.getLEDPatternFromFreight();
                if (o == null) {
                    if (currentColor != defaultColor) {
                        leds.setPattern(defaultColor);
                        currentColor = defaultColor;
                    }
                } else {
                    if (currentColor != o) {
                        leds.setPattern(o);
                        currentColor = o;
                    }
                }


            } else {
                intake.setMotorPower(0);
            }

            if (gamepad2.x) {
                spinner.setPowerBlueDirection();
            } else if (gamepad2.b) {
                spinner.setPowerRedDirection();
            } else {
                spinner.stop();
            }

            if (gamepad1.b) {
                driveTrain.setDrivePowerMult(0.3);
            }
            if (gamepad1.x) {
                driveTrain.setDrivePowerMult(1);

            }
            if (gamepad1.a) {
                driveTrain.setDrivePowerMult(0.6);
            }
        }
    }
}
