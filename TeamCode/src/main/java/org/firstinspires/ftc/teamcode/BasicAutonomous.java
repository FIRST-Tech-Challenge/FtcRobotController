package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Objects;
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
		POSITION,
		PLACE
	}
	YellowState yellowState;

	public enum Score{
		// TODO: do this later
	}

	String start_dist;
	String end_pos;
	Pose2d start_pos;
	String position;

	SampleMecanumDrive drive;
	TrajectorySequence purple_pixel;
	TrajectorySequence yellow_pixel;
	TrajectorySequence yellow_pixel_place;
	TrajectorySequence park;

	Servo yellowArm;

	private ElapsedTime runtime;

	private CameraPipeline cameraPipeline;
	private VisionPortal portal;

	double color;
	double center_line;
	double left_line;
	double right_line;

	public void init() {
		cameraPipeline = new CameraPipeline();

		yellowArm = hardwareMap.get(Servo.class, "bucket");
		runtime = new ElapsedTime();

		state = State.PLACE_PURPLE;
		yellowState = YellowState.DRIVE;

		drive = new SampleMecanumDrive(hardwareMap);

		color = -1.; // 1. for red, -1. for blue
		start_dist = "close"; // close or far, depending on start pos
		end_pos = "middle"; // either edge or middle, have to talk with alliance to get this value

		center_line = 14.5;
		left_line = 8.;
		right_line = 15.5;

		if (start_dist == "far") {
			center_line = -35;
			left_line = -39;
			right_line = -31;
		}
		start_pos = new Pose2d(center_line, -61*color, Math.toRadians(-90*color));

		drive.setPoseEstimate(start_pos);
		// Get the position of left, mid, or right
		portal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Goof"))
				.setCameraResolution(new Size(640, 480))
				.setCamera(BuiltinCameraDirection.BACK)
				.addProcessor(cameraPipeline)
				.enableLiveView(true)
				.build();

		cameraPipeline.setColor("blue");
	}

	public void start() {
		runtime.reset();
		while (runtime.time() < 1.5) {}

		position = cameraPipeline.getPropPosition();
		telemetry.addData("Position: ", position);
		telemetry.update();
		if (color == 1.) {
			if (Objects.equals(position, "left")) {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineToLinearHeading(new Pose2d(left_line, -30, Math.toRadians(0)))
						.lineTo(new Vector2d(right_line, -30))
						.lineTo(new Vector2d(center_line, -42))
						.turn(Math.toRadians(180))
						.build();
			} else if (Objects.equals(position, "mid")) {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(center_line, -31))
						.lineTo(new Vector2d(center_line, -42))
						.turn(Math.toRadians(-90))
						.build();
			} else {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineToLinearHeading(new Pose2d(right_line, -30*color, Math.toRadians(180)))
						.lineTo(new Vector2d(left_line, -30))
						.lineTo(new Vector2d(center_line, -42))
						.build();
			}
		} else {
			if (Objects.equals(position, "right")) {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineToLinearHeading(new Pose2d(left_line, 30, Math.toRadians(0)))
						.lineTo(new Vector2d(right_line, 30))
						.lineTo(new Vector2d(center_line, 42))
						.turn(Math.toRadians(180))
						.build();
			} else if (Objects.equals(position, "mid")) {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(center_line, 31))
						.lineTo(new Vector2d(center_line, 42))
						.turn(Math.toRadians(90))
						.build();
			} else {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineToLinearHeading(new Pose2d(right_line, 30, Math.toRadians(180)))
						.lineTo(new Vector2d(left_line, 30))
						.lineTo(new Vector2d(center_line, 42))
						.build();
			}
			portal.stopLiveView();
			portal.stopStreaming();
		}


		if (Objects.equals(start_dist, "close")) {
			yellow_pixel = drive.trajectorySequenceBuilder(purple_pixel.end())
					.lineTo(new Vector2d(50, -41*color))
					.build();
		} else {
			if (Objects.equals(position, "mid")) {
				yellow_pixel = drive.trajectorySequenceBuilder(purple_pixel.end())
						.lineTo(new Vector2d(-53, -42*color))
						.lineTo(new Vector2d(-53, -12*color))
						.lineTo(new Vector2d(38, -12*color))
						.lineTo(new Vector2d(50, -35*color))
						.build();
			} else {
				yellow_pixel = drive.trajectorySequenceBuilder(purple_pixel.end())
						.lineTo(new Vector2d(-35, -12*color))
						.lineTo(new Vector2d(38, -12*color))
						.lineTo(new Vector2d(50, -35*color))
						.build();
			}
		}

		if (Objects.equals(position, "left")) {
			yellow_pixel_place = drive.trajectorySequenceBuilder(yellow_pixel.end())
					.lineTo(new Vector2d(50, -36*color))
					.build();
		} else if (Objects.equals(position, "mid")) {
			yellow_pixel_place = drive.trajectorySequenceBuilder(yellow_pixel.end())
					.lineTo(new Vector2d(50, -42*color))
					.build();
		} else {
			yellow_pixel_place = drive.trajectorySequenceBuilder(yellow_pixel.end())
					.lineTo(new Vector2d(50, -49*color))
					.build();
		}

		if (Objects.equals(end_pos, "close")) {
			park = drive.trajectorySequenceBuilder(yellow_pixel_place.end())
					.lineTo(new Vector2d(50, -10*color))
					.build();
		}
		else {
			park = drive.trajectorySequenceBuilder(yellow_pixel_place.end())
					.lineTo(new Vector2d(50, -60*color))
					.build();
		}
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
							yellowState = YellowState.POSITION;
							drive.followTrajectorySequenceAsync(yellow_pixel_place);
						}
						break;
					case POSITION:
						if (!drive.isBusy()) {
							runtime.reset();
							yellowState = YellowState.PLACE;
						}
						break;
					case PLACE:
						yellowArm.setPosition(1);
						if (runtime.time() > 1) {
							yellowArm.setPosition(0.5);
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
