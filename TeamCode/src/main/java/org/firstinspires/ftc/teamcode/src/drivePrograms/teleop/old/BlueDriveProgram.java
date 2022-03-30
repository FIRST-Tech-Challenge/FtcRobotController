package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.old;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

/**
 * A Teleop for blue side Meets 1-3
 */
@Disabled
@TeleOp(name = "Blue Drive Program")
public class BlueDriveProgram extends TeleOpTemplate {
    private static final RevBlinkinLedDriver.BlinkinPattern defaultColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    public void opModeMain() throws InterruptedException {

        this.initAll();
        leds.setPattern(defaultColor);
        waitForStart();

        RevBlinkinLedDriver.BlinkinPattern currentColor = defaultColor;

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.gamepadControl(gamepad1, gamepad2);

            //Handles Linear Slide Control
            slide.setMotorPower(1 * gamepad2.left_stick_y);
            if (Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.01) {
                intake.setMotorPower(0.5 * (gamepad2.right_trigger - gamepad2.left_trigger));
                RevBlinkinLedDriver.BlinkinPattern o = outtake.getLEDPatternFromFreight();
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
                spinner.halt();
            }

            if (gamepad1.b) {
                driveTrain.setDrivePowerMultiplier(0.3);
            }
            if (gamepad1.x) {
                driveTrain.setDrivePowerMultiplier(1);

            }
            if (gamepad1.a) {
                driveTrain.setDrivePowerMultiplier(0.6);
            }
        }
    }
}
