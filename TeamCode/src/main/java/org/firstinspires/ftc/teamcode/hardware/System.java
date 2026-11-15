package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Hardware.END;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Hardware.INTAKE;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Hardware.LAUNCHER_LEFT;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Hardware.LAUNCHER_RIGHT;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Hardware.MIDDLE;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Hardware.TURRET;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.configuration.Constants;

import java.util.List;

public class System {
	private final List<LynxModule> allHubs;
	public Launcher launcher;
	public Rollers rollers;
	public Follower follower;
	public Turret turret;

	public System(HardwareMap hwMap) {
		allHubs = hwMap.getAll(LynxModule.class);

		// Required: drive motors + Pinpoint (inside createFollower) and the intake.
		// hwMap.get(...) stops the OpMode with an error if the name is not configured.
		follower = Constants.createFollower(hwMap);

		// Optional: hwMap.tryGet(...) returns null instead of crashing when missing from config.
		launcher = new Launcher(
				hwMap.tryGet(DcMotorEx.class, LAUNCHER_RIGHT),
				hwMap.tryGet(DcMotorEx.class, LAUNCHER_LEFT));

		rollers = new Rollers(
				hwMap.tryGet(DcMotor.class, INTAKE),
				hwMap.tryGet(DcMotor.class, MIDDLE),
				hwMap.tryGet(DcMotor.class, END));

		turret = new Turret(hwMap.tryGet(DcMotorEx.class, TURRET));

		setupHubs();
	}

	/** One line for telemetry showing which optional parts were found. */
	public String installedSummary() {
		return "Launcher " + (launcher.isInstalled() ? "yes" : "no")
				+ " | Middle " + (rollers.hasMiddle() ? "yes" : "no")
				+ " | End " + (rollers.hasEnd() ? "yes" : "no")
				+ " | Turret " + (turret.isInstalled() ? "yes" : "no");
	}

	public void update() {
		updateHubs();
		follower.update();
		launcher.update();
	}

	public void setupHubs() {
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
			hub.setConstant(Color.MAGENTA);
		}
	}

	public void updateHubs() {
		for (LynxModule hub : allHubs) {
			hub.clearBulkCache();
		}
	}
}