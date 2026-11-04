package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

@TeleOp(name = "TeleOp Main", group = "Main")
public class TeleOpGreen extends GreenBotBaseOpMode {

    private final int INITIAL_LAUNCHER_TARGET_VELOCITY = 1250;
    private final int MIN_LAUNCHER_TARGET_VELOCITY = 200;
    private final int MAX_LAUNCHER_TARGET_VELOCITY = 2000;

    private final double INTAKE_POWER = 1.0;
    private final double FEEDER_POWER = 1.0;

    private boolean intakeOn = true;
    private boolean intakeReversed = false;

    private double launcherTargetVelocity = INITIAL_LAUNCHER_TARGET_VELOCITY;

    @Override
    public void runOpMode() { // when driver hits INIT
        config();

        helpMenu();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            /////////////////////////
            // handle driving - driver 1
            /////////////////////////
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            drive.drive(forward, strafe, turn);

            /////////////////////////
            // handle intake - driver 1
            /////////////////////////
            if (gamepad1.leftBumperWasPressed()) {
                intakeOn = !intakeOn;
            }
            if (gamepad1.rightBumperWasPressed()) {
                intakeReversed = !intakeReversed;
            }
            double intakePower;
            intakePower = intakeOn ? INTAKE_POWER : 0.0;
            intakePower = intakeReversed ? -intakePower : intakePower;
            intake.setPower(intakePower);
            leftIntakeServo.setPower(intakePower);
            rightIntakeServo.setPower(intakePower);

            /////////////////////////
            // handle feeder - driver 2
            /////////////////////////
            if (gamepad2.leftBumperWasPressed()) {
                feeder.setPower(FEEDER_POWER);
            } else if (gamepad2.rightBumperWasPressed()) {
                feeder.setPower(-FEEDER_POWER);
            } else if (gamepad2.aWasPressed()){
                feeder.setPower(0.0);
            }

            /////////////////////////
            // handle turret - driver 2
            /////////////////////////
            turret.setPower(gamepad2.left_stick_x);

            /////////////////////////
            // handle launcher - driver 2
            // (runs continuously)
            /////////////////////////
            if (gamepad2.dpadUpWasPressed()) {
                launcherTargetVelocity += 100;
                if (launcherTargetVelocity > MAX_LAUNCHER_TARGET_VELOCITY) {
                    launcherTargetVelocity = MAX_LAUNCHER_TARGET_VELOCITY;
                }
            }
            if (gamepad2.dpadDownWasPressed()) {
                launcherTargetVelocity -= 100;
                if (launcherTargetVelocity < MIN_LAUNCHER_TARGET_VELOCITY) {
                    launcherTargetVelocity = MIN_LAUNCHER_TARGET_VELOCITY;
                }
            }
            launcher.setVelocity(launcherTargetVelocity);

            /////////////////////////
            // update telemetry
            /////////////////////////
            if (gamepad1.start || gamepad2.start) {
                helpMenu();
            } else {
                double launcherVelocity = launcher.getVelocity();
                telemetry.addData("Launcher - Target Velocity", launcherTargetVelocity);
                telemetry.addData("Launcher - Current Velocity", launcherVelocity);
            }
            telemetry.update();
        }
    }

    private void helpMenu() {
        telemetry.addData("Help Menu (this)", "Either Gamepad - Press Start");
        telemetry.addData("Drive controls", "Gamepad 1 - Standard Mecanum");
        telemetry.addData("Intake IN (Toggle)", "Gamepad 1 - Left Bumper");
        telemetry.addData("Intake OUT (Toggle)", "Gamepad 1 - Right Bumper");
        telemetry.addData("Feeder IN (Keep On)", "Gamepad 2 - Left Bumper");
        telemetry.addData("Feeder OUT (Keep On)", "Gamepad 2 - Right Bumper");
        telemetry.addData("Feeder OFF", "Gamepad 2 - A");
        telemetry.addData("Turret L/R", "Gamepad 2 - Left Stick L/R");
        telemetry.addData("Launcher +100", "Gamepad 2 - DPad Up");
        telemetry.addData("Launcher -100", "Gamepad 2 - DPad Down");
    }
}