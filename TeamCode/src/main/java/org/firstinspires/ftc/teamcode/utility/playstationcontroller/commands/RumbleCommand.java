package org.firstinspires.ftc.teamcode.utility.playstationcontroller.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class RumbleCommand extends CommandBase {
	private final GamepadEx gamepad;

	private final int durationMS;
	private final double intensity;

	public RumbleCommand(@NonNull GamepadEx gamepad) {
		this.gamepad    = gamepad;
		this.durationMS = 1000;
		this.intensity  = 1;
	}

	public RumbleCommand(@NonNull GamepadEx gamepad, int durationMS) {
		this.gamepad    = gamepad;
		this.durationMS = durationMS;
		this.intensity  = 1;
	}

	public RumbleCommand(@NonNull GamepadEx gamepad, int durationMS, double intensity) {
		this.gamepad    = gamepad;
		this.durationMS = durationMS;
		this.intensity  = intensity;
	}

	@Override public void execute() {
		gamepad.gamepad.rumble(intensity, intensity, durationMS);
	}

	@Override public boolean isFinished() {
		return true;
	}
}
