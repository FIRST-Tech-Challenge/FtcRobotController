package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Positions.STORED_POSE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configuration.Settings;
import org.firstinspires.ftc.teamcode.configuration.Sides;
import org.firstinspires.ftc.teamcode.hardware.System;

@Disabled // PHASE 1: hidden from the Driver Hub list. Delete this line to bring it back.
@Autonomous(name = "Close Auto", group = "Autonomous", preselectTeleOp = "TeleOp")
@Configurable // Panels
public class CloseAuto extends OpMode {

	public Follower follower; // Pedro Pathing follower instance
	private TelemetryManager panelsTelemetry; // Panels Telemetry instance
	private int pathState; // Current autonomous path state (state machine)
	private int launchSubState; // Sub-state for launch sequence (0-3)
	private Paths paths; // Paths defined in the Paths class
	private System system;
	private double rollStartTime; // Time when rollers started spinning
	private PathChain currentLaunchPath; // Path to use for current launch
	private boolean sideSelected = false; // Track if side has been selected

	@Override
	public void init() {
		system = new System(hardwareMap);
		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
		Drawing.init();
		follower = system.follower;

		// Side selection will happen in init_loop
		panelsTelemetry.debug("Status", "Waiting for side selection...");
		panelsTelemetry.update(telemetry);
	}

	@Override
	public void init_loop() {
		// Wait for side selection via gamepad
		if (!sideSelected) {
			if (gamepad1.a || gamepad2.a) {
				Settings.Positions.SIDE = Sides.BLUE;
				sideSelected = true;
				initializePaths();
			} else if (gamepad1.b || gamepad2.b) {
				Settings.Positions.SIDE = Sides.RED;
				sideSelected = true;
				initializePaths();
			} else {
				telemetry.addData("Side Selection", "Press A for BLUE, B for RED");
				telemetry.update();
				return;
			}
		}

		panelsTelemetry.debug("Side", Settings.Positions.SIDE.toString());
		panelsTelemetry.debug("Status", "Ready - Press Play");
		panelsTelemetry.update(telemetry);
	}

	/**
	 * Mirrors a pose across the field center (x=72)
	 * X: 144 - x, Heading: π - heading
	 */
	private static Pose mirrorPose(Pose pose) {
		return new Pose(
				144 - pose.getX(),
				pose.getY(),
				Math.PI - pose.getHeading()
		);
	}

	private void initializePaths() {
		// Mirror starting pose if RED side
		Pose startPose = Settings.Positions.Starting.close;
		if (Settings.Positions.SIDE == Sides.RED) {
			startPose = mirrorPose(startPose);
		}
		follower.setStartingPose(startPose);

		// Build paths (will mirror poses if RED side)
		paths = new Paths(follower, Settings.Positions.SIDE);

		pathState = 0; // Initialize state machine to start
		launchSubState = 0; // Initialize launch sub-state

		panelsTelemetry.debug("Status", "Initialized - " + Settings.Positions.SIDE + " side");
		Drawing.drawRobot(follower.getPose());
		Drawing.update();
	}

	@Override
	public void loop() {
		// Safety check - ensure paths are initialized
		if (paths == null) {
			telemetry.addData("Error", "Paths not initialized - select side in init");
			telemetry.update();
			return;
		}

		system.update();
		pathState = autonomousPathUpdate(); // Update autonomous state machine

		// Log values to Panels and Driver Station
		panelsTelemetry.debug("Path State", pathState);
		panelsTelemetry.debug("Side", Settings.Positions.SIDE.toString());
		panelsTelemetry.debug("X", follower.getPose().getX());
		panelsTelemetry.debug("Y", follower.getPose().getY());
		panelsTelemetry.debug("Heading", follower.getPose().getHeading());
		panelsTelemetry.update(telemetry);
		Drawing.drawDebug(follower);
		Drawing.update();
	}

	@Override
	public void stop() {
		STORED_POSE = follower.getPose();
	}

	/**
	 * Launch sequence helper function - handles spinning up, moving to launch,
	 * waiting for speed, running rollers, and stopping rollers.
	 * Returns true when launch sequence is complete, false otherwise.
	 */
	private boolean launch(PathChain launchPath) {
		switch (launchSubState) {
			case 0: // Start spinning up launcher and begin path
				system.launcher.setRPM(Settings.Launcher.CLOSE_VELOCITY);
				system.launcher.spinUp();
				follower.followPath(launchPath);
				launchSubState = 1;
				return false;

			case 1: // Moving to launch position (launcher spinning up in parallel)
				if (!follower.isBusy()) {
					launchSubState = 2; // Wait for launcher to reach speed
				}
				return false;

			case 2: // Wait until launcher is at speed
				if (system.launcher.isAtSpeed()) {
					// Start all rollers spinning
					system.rollers.intakeIn();
					system.rollers.middleIn();
					system.rollers.endIn();
					system.rollers.assistIn();
					rollStartTime = getRuntime();
					launchSubState = 3;
				}
				return false;

			case 3: // Rollers spinning - wait for ROLL_TIME
				if (getRuntime() - rollStartTime >= Settings.Auto.ROLL_TIME) {
					// Stop all rollers
					system.rollers.intakeStop();
					system.rollers.middleStop();
					system.rollers.endStop();
					system.rollers.assistStop();
					launchSubState = 0; // Reset for next launch
					return true; // Launch complete
				}
				return false;

			default:
				launchSubState = 0;
				return false;
		}
	}

	public int autonomousPathUpdate() {
		// State machine for Close Auto - easy to extend
		switch (pathState) {
			case 0: // First launch - from starting position
				currentLaunchPath = paths.toLaunchPosition;
				pathState = 4; // Enter launch sequence
				break;

			case 1: // Go to PRESET_3_PREP
				follower.followPath(paths.toPreset3Prep);
				system.rollers.intakeIn();
				system.rollers.middleIn();
				pathState = 2;
				break;

			case 2: // Moving to PRESET_3_PREP
				if (!follower.isBusy()) {
					follower.followPath(paths.toPreset3End, 0.3, true);
					pathState = 3;
				}
				break;

			case 3: // Moving to PRESET_3_END
				if (!follower.isBusy()) {
					// Second launch - from PRESET_3_END
					system.rollers.intakeStop();
					system.rollers.middleStop();
					currentLaunchPath = paths.toLaunchPosition2;
					pathState = 4; // Enter launch sequence
				}
				break;

			case 4: // Launch sequence (reusable for all three launches)
				if (launch(currentLaunchPath)) {
					// Launch complete - determine next state
					if (currentLaunchPath == paths.toLaunchPosition) {
						pathState = 1; // First launch done, go to PRESET_3
					} else if (currentLaunchPath == paths.toLaunchPosition2) {
						pathState = 5; // Second launch done, go to PRESET_2
					} else if (currentLaunchPath == paths.toLaunchPosition3) {
						pathState = 8; // Third launch done, go to park
					}
				}
				break;

			case 5: // Go to PRESET_2_PREP
				follower.followPath(paths.toPreset2Prep);
				pathState = 6;
				break;

			case 6: // Moving to PRESET_2_PREP
				if (!follower.isBusy()) {
					system.rollers.intakeIn();
					system.rollers.middleIn();
					follower.followPath(paths.toPreset2End, 0.3, true);
					pathState = 7;
				}
				break;

			case 7: // Moving to PRESET_2_END
				if (!follower.isBusy()) {
					// Third launch - from PRESET_2_END
					system.rollers.intakeStop();
					system.rollers.middleStop();
					currentLaunchPath = paths.toLaunchPosition3;
					pathState = 4; // Enter launch sequence
				}
				break;

			case 8: // Go to PARK
				follower.followPath(paths.toPark);
				pathState = 9;
				break;

			case 9: // Moving to PARK
				if (!follower.isBusy()) {
					pathState = 10; // Done
				}
				break;

			case 10: // Done
				break;

			default:
				pathState = 0; // Reset to start if invalid state
				break;
		}

		return pathState;
	}

	public static class Paths {

		public PathChain toLaunchPosition;
		public PathChain toPreset3Prep;
		public PathChain toPreset3End;
		public PathChain toLaunchPosition2;
		public PathChain toPreset2Prep;
		public PathChain toPreset2End;
		public PathChain toLaunchPosition3;
		public PathChain toPark;

		public Paths(Follower follower, Sides side) {
			// Helper to get pose (mirrored if RED)
			java.util.function.Function<Pose, Pose> getPose = pose -> side == Sides.RED
					? mirrorPose(pose)
					: pose;

			// Path from starting position to launch position
			toLaunchPosition = follower
					.pathBuilder()
					.addPath(
							new BezierLine(
									getPose.apply(Settings.Positions.Starting.close),
									getPose.apply(Settings.Positions.Launch.close)))
					.setTangentHeadingInterpolation()
					.build();

			// Path from launch position to PRESET_3_PREP
			toPreset3Prep = follower
					.pathBuilder()
					.addPath(
							new BezierLine(
									getPose.apply(Settings.Positions.Launch.close),
									getPose.apply(Settings.Positions.Preset3.PREP)))
					.setLinearHeadingInterpolation(getPose.apply(Settings.Positions.Launch.close).getHeading(),
							getPose.apply(Settings.Positions.Preset3.PREP).getHeading())
					.setTangentHeadingInterpolation()
					.build();

			// Path from PRESET_3_PREP to PRESET_3_END
			toPreset3End = follower
					.pathBuilder()
					.addPath(
							new BezierLine(
									getPose.apply(Settings.Positions.Preset3.PREP),
									getPose.apply(Settings.Positions.Preset3.END)))
					.setTangentHeadingInterpolation()
					.build();

			// Path from PRESET_3_END back to launch position
			toLaunchPosition2 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(
									getPose.apply(Settings.Positions.Preset3.END),
									getPose.apply(Settings.Positions.Launch.close)))
					.setLinearHeadingInterpolation(getPose.apply(Settings.Positions.Preset3.END).getHeading(),
							getPose.apply(Settings.Positions.Launch.close).getHeading())
					.build();

			// Path from launch position to PRESET_2_PREP
			toPreset2Prep = follower
					.pathBuilder()
					.addPath(
							new BezierLine(
									getPose.apply(Settings.Positions.Launch.close),
									getPose.apply(Settings.Positions.Preset2.PREP)))
					.setLinearHeadingInterpolation(getPose.apply(Settings.Positions.Launch.close).getHeading(),
							getPose.apply(Settings.Positions.Preset2.PREP).getHeading())
					.build();

			// Path from PRESET_2_PREP to PRESET_2_END
			toPreset2End = follower
					.pathBuilder()
					.addPath(
							new BezierLine(
									getPose.apply(Settings.Positions.Preset2.PREP),
									getPose.apply(Settings.Positions.Preset2.END)))
					.setTangentHeadingInterpolation()
					.build();

			// Path from PRESET_2_END back to launch position
			toLaunchPosition3 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(
									getPose.apply(Settings.Positions.Preset2.END),
									getPose.apply(Settings.Positions.Launch.close)))
					.setLinearHeadingInterpolation(getPose.apply(Settings.Positions.Preset2.END).getHeading(),
							getPose.apply(Settings.Positions.Launch.close).getHeading())
					.build();

			// Path from launch position to PARK
			toPark = follower
					.pathBuilder()
					.addPath(
							new BezierLine(
									getPose.apply(Settings.Positions.Launch.close),
									getPose.apply(Settings.Positions.Park.close)))
					.setTangentHeadingInterpolation()
					.build();
		}

		/**
		 * Mirrors a pose across the field center (x=72)
		 * X: 144 - x, Heading: π - heading
		 */
		private static Pose mirrorPose(Pose pose) {
			return new Pose(
					144 - pose.getX(),
					pose.getY(),
					Math.PI - pose.getHeading()
			);
		}
	}
}