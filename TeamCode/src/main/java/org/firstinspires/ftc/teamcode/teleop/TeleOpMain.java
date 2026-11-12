package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

@TeleOp(name = "TeleOp Main", group = "Main")
public class TeleOpMain extends GreenBotBaseOpMode {

    private final double INITIAL_LAUNCHER_TARGET_VELOCITY = 1250;
    private final double LAUNCHER_VELOCITY_STEP = 100;

    private final double INTAKE_POWER = 1.0;
    private final double FEEDER_POWER = 1.0;

    private final double SPEED_FACTOR = 0.5;

    private final double TURRET_NUDGE_POWER = 0.25;

    private boolean intakeOn = true;
    private boolean wasOnTarget = false;
    private boolean intakeReversed = false;

    @Override
    public void runOpMode() { // when driver hits INIT
        config();

        // The turret should be lined up facing straight forward before INIT; that becomes 0.
        // TODO: once an auto moves the turret, skip this when TeleOp follows that auto.
        aimer.resetTurretEncoder();

        // Pick the alliance during INIT (it decides which hive tags the aimer looks for).
        while (opModeInInit()) {
            if (gamepad1.x || gamepad2.x) aimer.setBlueAlliance(true);
            if (gamepad1.b || gamepad2.b) aimer.setBlueAlliance(false);
            telemetry.addData("Alliance", "%s   (X = BLUE, B = RED)",
                    aimer.isBlueAlliance() ? "BLUE" : "RED");
            telemetry.addLine("Turret encoder zeroed -- it should be facing straight forward.");
            helpMenu();
            telemetry.update();
        }

        waitForStart();
        wedgeServo.setPosition(WEDGE_UP);
        aimer.start();
        launcher.setTargetVelocity(INITIAL_LAUNCHER_TARGET_VELOCITY);

        while (opModeIsActive()) {
            /////////////////////////
            // handle driving - driver 1
            /////////////////////////
            double forward = -gamepad1.left_stick_y * SPEED_FACTOR;
            double strafe = gamepad1.left_stick_x * SPEED_FACTOR;
            double turn = -gamepad1.right_stick_x * SPEED_FACTOR;
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

            /////////////////////////
            // handle wedge - driver 1
            /////////////////////////
            if (gamepad1.dpadUpWasPressed()) {
                wedgeServo.setPosition(WEDGE_UP);
            }
            if (gamepad1.dpadDownWasPressed()) {
                wedgeServo.setPosition(WEDGE_DOWN);
            }

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
            // D-pad L/R nudges (within the soft limits) and overrides auto-aim.
            // Hold Y to auto-aim at the hive.
            /////////////////////////
            aimer.update();
            if (gamepad2.dpad_left || gamepad2.dpad_right) {
                aimer.moveTurret(gamepad2.dpad_right ? TURRET_NUDGE_POWER : -TURRET_NUDGE_POWER);
            } else if (gamepad2.y) {
                aimer.aim();
            } else {
                aimer.stopTurret();
            }

            // Buzz driver 2's controller once when the turret first lines up.
            boolean onTarget = aimer.isOnTarget();
            if (onTarget && !wasOnTarget) {
                gamepad2.rumble(200);
            }
            wasOnTarget = onTarget;

            /////////////////////////
            // handle launcher - driver 2
            // (runs continuously)
            /////////////////////////
            if (gamepad2.dpadUpWasPressed()) {
                launcher.changeTargetVelocity(LAUNCHER_VELOCITY_STEP);
            }
            if (gamepad2.dpadDownWasPressed()) {
                launcher.changeTargetVelocity(-LAUNCHER_VELOCITY_STEP);
            }

            /////////////////////////
            // update LEDs
            /////////////////////////
            updateLights();

            /////////////////////////
            // update telemetry
            /////////////////////////
            if (gamepad1.start || gamepad2.start) {
                helpMenu();
            } else {
                launcher.showStatus();
                aimer.showStatus();
                drive.showPose();
            }
            telemetry.update();
        }

        drive.stopDriving();
        aimer.stop();
        launcher.stop();
    }

    private void helpMenu() {
        telemetry.addData("Help Menu (this)", "Either Gamepad - Hold Start");

        telemetry.addData("----------", "-----------");
        telemetry.addData("Drive controls", "Gamepad 1 - Standard Mecanum");
        telemetry.addData("Intake ON/OFF (Toggle)", "Gamepad 1 - Left Bumper");
        telemetry.addData("Intake Reverse (Toggle)", "Gamepad 1 - Right Bumper");
        telemetry.addData("Wedge UP", "Gamepad 1 - DPad Up");
        telemetry.addData("Wedge DOWN", "Gamepad 1 - DPad Down");
        telemetry.addData("----------", "-----------");

        telemetry.addData("Feeder IN (Keep On)", "Gamepad 2 - Left Bumper");
        telemetry.addData("Feeder OUT (Keep On)", "Gamepad 2 - Right Bumper");
        telemetry.addData("Feeder OFF", "Gamepad 2 - A");
        telemetry.addData("Turret L/R", "Gamepad 2 - DPad Left/Right");
        telemetry.addData("Turret Auto-Aim (Hold)", "Gamepad 2 - Y");
        telemetry.addData("Launcher +100", "Gamepad 2 - DPad Up");
        telemetry.addData("Launcher -100", "Gamepad 2 - DPad Down");
    }
}