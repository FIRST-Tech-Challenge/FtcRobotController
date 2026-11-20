package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.System;

@Autonomous(name = "Park Auto", group = "Auto")
public class ParkAuto extends LinearOpMode {
	// Distance in inches to move out of parking and back
	private static final double MOVE_OUT_INCHES = 6.0;

	// FTC Autonomous is 30s; start returning early enough to finish safely
	private static final double RETURN_AT_SECONDS = 26.5;

	private System system;

	private void driveTo(Pose from, Pose to) {
		PathChain path = system.follower.pathBuilder()
				.addPath(new BezierLine(from, to))
				.setConstantHeadingInterpolation(from.getHeading())
				.build();
		system.follower.followPath(path, true);
		while (opModeIsActive() && system.follower.isBusy()) {
			system.update();
		}
	}

	@Override
	public void runOpMode() {
		system = new System(hardwareMap);

		Pose start = new Pose(0, 0, 0);
		Pose outPose = new Pose(MOVE_OUT_INCHES, 0, 0);

		system.follower.setStartingPose(start);

		telemetry.addData("Status", "Ready");
		telemetry.update();
		waitForStart();

		ElapsedTime autoTimer = new ElapsedTime();

		// 1. Move forward out of parking
		if (opModeIsActive()) {
			driveTo(start, outPose);
		}

		// 2. Stay put until near the end of Autonomous
		while (opModeIsActive() && autoTimer.seconds() < RETURN_AT_SECONDS) {
			system.update();
			telemetry.addData("Status", "Holding");
			telemetry.addData("Elapsed (s)", "%.1f", autoTimer.seconds());
			telemetry.update();
		}

		// 3. Drive straight back to parking pose
		if (opModeIsActive()) {
			driveTo(outPose, start);
		}

		while (opModeIsActive()) {
			system.update();
			telemetry.addData("Status", "Parked");
			telemetry.update();
		}
	}
}