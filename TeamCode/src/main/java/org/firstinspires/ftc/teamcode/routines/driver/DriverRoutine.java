package org.firstinspires.ftc.teamcode.routines.driver;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.routines.Routine;
import org.firstinspires.ftc.teamcode.subsystems.System;

@TeleOp(name = "Driver")
public class DriverRoutine extends Routine {
	public System system = new System(this);

	@Override
	public void onInit() {
		super.onInit();
	}

	@Override
	public void onStart() {
		super.onStart();
	}

	@Override
	public void onExit() {
		super.onExit();
		system.onExit();
	}
}
