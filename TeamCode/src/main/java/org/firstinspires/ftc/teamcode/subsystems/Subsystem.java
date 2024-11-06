package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.routines.Routine;

public abstract class Subsystem {
	protected Routine routine;
	protected Telemetry telemetry;

	public Subsystem(Routine routine) {
		this.routine = routine;
	}

	public void onInit() {
		this.telemetry = routine.telemetry;
	}

	public void onStart() {}

	public void onExit() {}

	protected boolean isStopRequested() {
		return routine.isStopRequested();
	}
}
