package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

	String position;

	@Override
	public void init()
	{
		// Get the position of left, mid, or right
		position = "mid";
	}

	@Override
	public void loop()
	{
		switch(state)
		{
			case PLACE_PURPLE:
				// places the purple pixel depending on the position
				if(position == "left")
				{

				}else if(position == "mid")
				{

				}else if (position == "right")
				{

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
