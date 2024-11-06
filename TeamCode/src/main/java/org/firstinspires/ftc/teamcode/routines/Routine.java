package org.firstinspires.ftc.teamcode.routines;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Routine extends LinearOpMode {
	public ExecutorService executor = Executors.newFixedThreadPool(4);
	private boolean exited = false;

	@Override
	public void runOpMode() {
		// Run initialization code
		onInit();

		// Wait for start in case of initialization code ending early
		waitForStart();
		if (isStopRequested()) { // Terminate code if cancelled
			onExit();
			return;
		}

		// Run routine code
		executor.submit(() -> {
			onStart();
			if (!this.exited) onExit();
		});

		// Handle unexpected exits
		do sleep(100);
		while (opModeIsActive() && !this.exited);
		if (!this.exited) onExit();
	}

	public void onInit() {}
	public void onStart() {}
	public void onExit() {
		this.exited = true;
		executor.shutdownNow();
	}
}
