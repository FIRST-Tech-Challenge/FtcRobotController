package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Positions.STORED_POSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.configuration.Sides;
import org.firstinspires.ftc.teamcode.hardware.Launcher;
import org.firstinspires.ftc.teamcode.hardware.System;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "OpMode")
public class TeleOp extends LinearOpMode {
	private final ElapsedTime runtime = new ElapsedTime();
	private System system;

	@Override
	public void runOpMode() {
		system = new System(hardwareMap);

		telemetry.addData("Status", "Initialized");
		telemetry.addData("Installed", system.installedSummary());
		telemetry.update();

		waitForStart();
		system.follower.setStartingPose(STORED_POSE);
		system.follower.startTeleOpDrive();
		runtime.reset();

		// Run until the end of the match (driver presses STOP)
		while (opModeIsActive()) {
			system.update();

			// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
			double axial = -gamepad1.left_stick_y;  // Pushing stick forward gives negative value
			double lateral = gamepad1.left_stick_x;
			double yaw = -gamepad1.right_stick_x;

			if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
				if (gamepad1.a) {
					system.follower.holdPoint(Settings.Positions.SIDE == Sides.BLUE ? Settings.Positions.Launch.close : Settings.Positions.Launch.close.mirror());
				}
				if (gamepad1.b) {
					system.follower.holdPoint(Settings.Positions.SIDE == Sides.BLUE ? Settings.Positions.Launch.far : Settings.Positions.Launch.far.mirror());
				}
				if (gamepad1.x) {
					system.follower.holdPoint(Settings.Positions.SIDE == Sides.BLUE ? Settings.Positions.human.HUMAN : Settings.Positions.human.HUMAN.mirror());
				}
				if (gamepad1.y) {
					system.follower.holdPoint(Settings.Positions.SIDE == Sides.BLUE ? Settings.Positions.Endgame.PARK : Settings.Positions.Endgame.PARK.mirror());
				}
			} else {
				if (!system.follower.isTeleopDrive()) {
					system.follower.startTeleOpDrive();
				}
				system.follower.setTeleOpDrive(axial, lateral, yaw);
			}

			// Intake control — use right trigger for intake in, left trigger for reverse
			system.rollers.setIntakePower(gamepad2.right_trigger - gamepad2.left_trigger);

			// Turret: gamepad 2 right stick left/right
			double turretInput = Math.abs(gamepad2.right_stick_x) < 0.05 ? 0 : gamepad2.right_stick_x;
			system.turret.setPower(turretInput);

			// Outtake control — press 'A' to run low goal, 'X' for high goal
			if (gamepad2.a) {
				system.launcher.setRPM(Settings.Launcher.CLOSE_VELOCITY);
			} else if (gamepad2.b) {
				system.launcher.setRPM(-Settings.Launcher.CLOSE_VELOCITY);
			}

			if (gamepad2.x || gamepad2.y) {
				system.launcher.setRPM(Settings.Launcher.FAR_VELOCITY);
			}

			if (gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y) {
				system.launcher.spinUp();
			} else {
				system.launcher.spinDown();
			}

			// Middle roller (gamepad 2 d-pad) and end roller (gamepad 1 d-pad)
			if (gamepad2.dpad_up) {
				system.rollers.middleIn();    // Forward
			} else if (gamepad2.dpad_down) {
				system.rollers.middleOut();   // Reverse
			} else if (gamepad2.dpad_left || gamepad2.dpad_right) {
				system.rollers.middleStop();  // Stop
			}

			if (gamepad1.dpad_up) {
				system.rollers.endIn();
			} else if (gamepad1.dpad_down) {
				system.rollers.endOut();
			} else if (gamepad1.dpad_left || gamepad1.dpad_right) {
				system.rollers.endStop();
			}

			// Telemetry diagnostics
			telemetry.addData("Pose", system.follower.getPose());

			if (system.rollers.hasIntake()) {
				telemetry.addData("Intake Power", system.rollers.intake.getPower());
			} else {
				telemetry.addData("Intake Power", "NOT FOUND");
			}

			if (system.rollers.hasMiddle()) {
				telemetry.addData("Middle Wheels Power", system.rollers.getMiddlePower());
			}
			if (system.rollers.hasEnd()) {
				telemetry.addData("End Wheels Power", system.rollers.getEndPower());
			}
			if (system.launcher.isInstalled()) {
				telemetry.addData("Outtake Target", Launcher.ticksPerSecToRPM(system.launcher.targetTPS));
				telemetry.addData("Outtake Actual", system.launcher.getRPM());
			}
			if (system.turret.isInstalled()) {
				telemetry.addData("Turret Position", system.turret.getPosition());
			} else {
				telemetry.addData("Turret", "NOT FOUND");
			}
			telemetry.addData("Installed", system.installedSummary());
			telemetry.addData("Status", "Run Time: " + runtime);
			Drawing.drawDebug(system.follower);
			Drawing.update();
			telemetry.update();
		}
	}
}