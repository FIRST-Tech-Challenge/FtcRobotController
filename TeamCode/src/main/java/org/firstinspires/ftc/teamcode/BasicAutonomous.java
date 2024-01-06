package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Vector;

@Autonomous
public class BasicAutonomous extends OpMode
{
	public enum State{
		PLACE_PURPLE,
		PLACE_YELLOW,
		SCORE,
		PARK
	}
	State state = State.PLACE_PURPLE;

	public enum YellowState{
		DRIVE,
		DETECT,
		PLACE
	}
	YellowState yellowState = YellowState.DRIVE;

	public enum Score{
		// TODO: do this later
	}

	double color = 1.; // 1. for red, -1. for blue
	String start_dist = "close"; // close or far, depending on start pos
	String end_pos = "edge"; // either edge or middle, have to talk with alliance to get this value

	Pose2d start_pos;

	String position;
	SampleMecanumDrive drive;
	TrajectorySequence purple_pixel;

	@Override
	public void init()
	{
		double center_line = 70./6.;
		double left_line = 9.;
		double right_line = 14.;
		if (start_dist == "close") {
			start_pos = new Pose2d(70./6., -61*color, Math.toRadians(-90*color));
		} else if (start_dist == "far") {
			start_pos = new Pose2d(-35, -61*color, Math.toRadians(-90*color));
		}

		drive = new SampleMecanumDrive(hardwareMap);
		drive.setPoseEstimate(start_pos);

		// Get the position of left, mid, or right
		position = "mid";

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
						.lineTo(new Vector2d(center_line, -32.5))
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
						.lineTo(new Vector2d(70./6., 32.5))
						.turn(Math.toRadians(-90))
						.lineTo(new Vector2d(9, 30))
						.lineTo(new Vector2d(14, 30))
						.lineTo(new Vector2d(70./6., 42))
						.turn(Math.toRadians(180))
						.build();
			} else if (position == "mid") {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(70./6., 32.5))
						.lineTo(new Vector2d(70./6., 42))
						.turn(Math.toRadians(90))
						.build();
			} else if (position == "left") {
				purple_pixel = drive.trajectorySequenceBuilder(start_pos)
						.lineTo(new Vector2d(70./6., 32.5))
						.turn(Math.toRadians(90))
						.lineTo(new Vector2d(14, 30))
						.lineTo(new Vector2d(9, 30))
						.lineTo(new Vector2d(70./6., 42))
						.build();
			}
		}


		if (start_dist == "close") {

		}

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
				}

				break;
			case PLACE_YELLOW:
				switch(yellowState)
				{
					case DRIVE:
						// drive to backboard
						break;
					case DETECT:
						// detect which position it is
						break;
					case PLACE:
						// place the pixel
						break;
				}
				break;
			case SCORE:
				state = State.PARK;
				break;
			case PARK:
				// goto park position
				break;
		}
	}
}
