package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Vector;

@Autonomous
public class BasicAutonomous extends OpMode
{
	public enum State{
		PLACE_PURPLE,
		PLACE_YELLOW,
		SCORE,
		PARK,
		Idle
	}
	State state;

	public enum YellowState{
		DRIVE,
		PLACE
	}
	YellowState yellowState;

	public enum Score{
		// TODO: do this later
	}

	double color;
	String start_dist;
	String end_pos;

	Pose2d start_pos;

	String position;
	SampleMecanumDrive drive;
	TrajectorySequence purple_pixel;
	TrajectorySequence yellow_pixel;
	TrajectorySequence park;

	Servo yellowArm = hardwareMap.get(Servo.class, "bucket");

	private ElapsedTime runtime = new ElapsedTime();

	private CameraPipeline cameraPipeline;
	private VisionPortal portal;

	@Override
	public void init()
	{
		state = State.PLACE_PURPLE;
		yellowState = YellowState.DRIVE;

		color = 1.; // 1. for red, -1. for blue
		start_dist = "close"; // close or far, depending on start pos
		end_pos = "edge"; // either edge or middle, have to talk with alliance to get this value

		double center_line = 70./6.;
		double left_line = 9.;
		double right_line = 14.;

		if (start_dist == "close") {
			start_pos = new Pose2d(70./6., -61*color, Math.toRadians(-90*color));
		} else if (start_dist == "far") {
			start_pos = new Pose2d(-35, -61*color, Math.toRadians(-90*color));
			center_line = -35;
			left_line = -38;
			right_line = -33;
		}

		drive = new SampleMecanumDrive(hardwareMap);
		drive.setPoseEstimate(start_pos);

		cameraPipeline.setColor("red");

		// Get the position of left, mid, or right
		portal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
				.setCameraResolution(new Size(640, 480))
				.setCamera(BuiltinCameraDirection.BACK)
				.addProcessor(cameraPipeline)
				.build();

		position = cameraPipeline.getPropPosition();

		// defines the trajectory for placing purple pixel
		if (color == 1) {
			if (position == "left") {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(center_line, -32.5))
						.turn(Math.toRadians(90))
						.lineTo(new Vector2d(left_line, -30))
						.lineTo(new Vector2d(right_line, -30))
						.lineTo(new Vector2d(center_line, -42))
						.turn(Math.toRadians(180))
						.build();
			} else if (position == "mid") {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(center_line, -31))
						.lineTo(new Vector2d(center_line, -42))
						.turn(Math.toRadians(-90))
						.build();
			} else if (position == "right") {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(center_line, -32.5))
						.turn(Math.toRadians(-90))
						.lineTo(new Vector2d(right_line, -30))
						.lineTo(new Vector2d(left_line, -30))
						.lineTo(new Vector2d(center_line, -42))
						.build();
			}
		} else if (color == -1) {
			if (position == "right") {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(center_line, 32.5))
						.turn(Math.toRadians(-90))
						.lineTo(new Vector2d(left_line, 30))
						.lineTo(new Vector2d(right_line, 30))
						.lineTo(new Vector2d(center_line, 42))
						.turn(Math.toRadians(180))
						.build();
			} else if (position == "mid") {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(center_line, 32.5))
						.lineTo(new Vector2d(center_line, 42))
						.turn(Math.toRadians(90))
						.build();
			} else if (position == "left") {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(center_line, 32.5))
						.turn(Math.toRadians(90))
						.lineTo(new Vector2d(right_line, 30))
						.lineTo(new Vector2d(left_line, 30))
						.lineTo(new Vector2d(center_line, 42))
						.build();
			}
		}


		if (start_dist == "close") {
			yellow_pixel = drive.trajectorySequenceBuilder(purple_pixel.end())
					.lineTo(new Vector2d(49, -38*color))
					.build();
		} else if (start_dist == "far") {
			if (position == "mid") {
				yellow_pixel = drive.trajectorySequenceBuilder(purple_pixel.end())
						.lineTo(new Vector2d(-35, -35))
						.lineTo(new Vector2d(49, -35))
						.build();
			} else {
				yellow_pixel = drive.trajectorySequenceBuilder(purple_pixel.end())
						.lineTo(new Vector2d(-35, -12))
						.lineTo(new Vector2d(38, -12))
						.lineTo(new Vector2d(49, -35))
						.build();
			}
		}

		// Set a place to park
		park = drive.trajectorySequenceBuilder(yellow_pixel.end())
				.waitSeconds(1.0)
				.build();

	}

	@Override
	public void start() {
		drive.followTrajectorySequenceAsync(purple_pixel);
	}

	@Override
	public void loop()
	{
		switch(state)
		{
			case PLACE_PURPLE:
				if (!drive.isBusy()) {
					state = State.PLACE_YELLOW;
					drive.followTrajectorySequenceAsync(yellow_pixel);
				}

				break;
			case PLACE_YELLOW:
				switch(yellowState)
				{
					case DRIVE:
						if (!drive.isBusy()) {
							yellowState = YellowState.PLACE;
							runtime.reset();
						}
						break;
					case PLACE:
						yellowArm.setPosition(1);
						if (runtime.time() > 1) {
							yellowArm.setPosition(0);
							state = State.SCORE;
						}
						break;
				}
				break;
			case SCORE:
				state = State.PARK;
				drive.followTrajectorySequenceAsync(park);
				break;
			case PARK:
				if (!drive.isBusy()) {
					state = State.Idle;
				}
				break;
			case Idle:
				break;
		}

		drive.update();
	}
}
