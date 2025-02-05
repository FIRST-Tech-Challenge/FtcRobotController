package org.firstinspires.ftc.teamcode.utility.playstationcontroller.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ChangeLEDColorCommand extends CommandBase {
	private final GamepadEx gamepad;
	private final int       red, green, blue;
	private final int       durationMS;


	/**
	 * Changes the color of the PS4/PS5 controller LED for the specified duration
	 * @param gamepad    The PS4/PS5 controller to change the color of.
	 * @param red        The red component of the color
	 * @param green      The green component of the color
	 * @param blue       The blue component of the color
	 * @param durationMS The duration of the color change
	 */
	public ChangeLEDColorCommand(
			@NonNull GamepadEx gamepad,
			int red,
			int green,
			int blue,
			int durationMS
	) {
		this.gamepad    = gamepad;
		this.red        = red;
		this.green      = green;
		this.blue       = blue;
		this.durationMS = durationMS;
	}

	/**
	 * Changes the color of the PS4/PS5 controller LED.
	 * @param gamepad The controller to change the color of
	 * @param red     The red component of the color
	 * @param green   The green component of the color
	 * @param blue    The blue component of the color
	 */
	public ChangeLEDColorCommand(
			@NonNull GamepadEx gamepad,
			int red,
			int green,
			int blue
	) {
		this.gamepad    = gamepad;
		this.red        = red;
		this.green      = green;
		this.blue       = blue;
		this.durationMS = Gamepad.LED_DURATION_CONTINUOUS;

	}

	@Override public void execute() {
		gamepad.gamepad.setLedColor(red, green, blue, durationMS);
	}

	@Override public boolean isFinished() {
		return true;
	}
}
