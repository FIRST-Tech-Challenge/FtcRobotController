package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.FAR_VELOCITY;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.MAX_SPEED_ERROR;
import static org.firstinspires.ftc.teamcode.configuration.Settings.Launcher.TICKS_PER_REVOLUTION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Two-motor flywheel launcher.
 *
 * PHASE 1: the launcher is not on the robot yet. If either motor is missing from the
 * robot configuration, System passes null and this class becomes "not installed":
 * every method is safe to call and simply does nothing. Once both motors are wired
 * and added to the configuration, it works again with no code change.
 */



public class Launcher {
	private static final double VELOCITY_ALPHA = 0.15; // EMA smoothing factor (0-1), lower = more smoothing
	private final DcMotorEx rightMotor;
	private final DcMotorEx leftMotor;
	private final boolean installed;
	public double targetTPS = 0;
	private LauncherState state = LauncherState.IDLE;
	private double averagedRightRPM = 0;
	private double averagedLeftRPM = 0;

	public Launcher(
			DcMotorEx launcherRight,
			DcMotorEx launcherLeft) {

		this.rightMotor = launcherRight;
		this.leftMotor = launcherLeft;
		this.installed = launcherRight != null && launcherLeft != null;

		if (installed) {
			rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

			leftMotor.setDirection(DcMotor.Direction.FORWARD);
			rightMotor.setDirection(DcMotor.Direction.REVERSE);
		}

		setRPM(FAR_VELOCITY);
	}

	/** True when both flywheel motors were found in the robot configuration. */
	public boolean isInstalled() {
		return installed;
	}

	public static double rpmToTicksPerSec(double rpm) {
		return rpm * TICKS_PER_REVOLUTION / 60.0;
	}

	/**
	 * Converts motor velocity in ticks per second to RPM.
	 * Use this when reading motor velocity and displaying as RPM.
	 *
	 * @param ticksPerSec velocity in ticks per second
	 * @return revolutions per minute
	 */
	public static double ticksPerSecToRPM(double ticksPerSec) {
		return ticksPerSec / TICKS_PER_REVOLUTION * 60.0;
	}

	public final void update() {
		if (!installed) return;
		// Update time-averaged velocity readings using exponential moving average
		// This smooths out noise from instantaneous velocity readings
		double currentRightRPM = ticksPerSecToRPM(rightMotor.getVelocity());
		double currentLeftRPM = ticksPerSecToRPM(leftMotor.getVelocity());

		// Apply exponential moving average: new = alpha * current + (1 - alpha) * old
		// This naturally handles initialization (if averaged is 0, it will gradually
		// build up)
		averagedRightRPM = VELOCITY_ALPHA * currentRightRPM + (1 - VELOCITY_ALPHA) * averagedRightRPM;
		averagedLeftRPM = VELOCITY_ALPHA * currentLeftRPM + (1 - VELOCITY_ALPHA) * averagedLeftRPM;
	}

	public void spinUp() {
		state = LauncherState.ACTIVE;
		if (!installed) return;
		leftMotor.setVelocity(targetTPS);
		rightMotor.setVelocity(targetTPS);
	}

	public void spinDown() {
		state = LauncherState.IDLE;
		if (!installed) return;
		leftMotor.setVelocity(0);
		rightMotor.setVelocity(0);
	}

	/**
	 * Gets the current RPM using time-averaged readings from both motors.
	 * Returns the average of the two motors' smoothed velocity readings.
	 * Always 0 when the launcher is not installed.
	 */
	public double getRPM() {
		return (averagedRightRPM + averagedLeftRPM) / 2.0;
	}

	public void setRPM(double rpm) {
		targetTPS = rpmToTicksPerSec(rpm);
	}

	/**
	 * Checks if the launcher is at the target speed.
	 * Uses time-averaged velocity readings and checks both motors separately
	 * to ensure both are within tolerance, preventing false positives from
	 * averaging out errors between motors.
	 *
	 * When the launcher is not installed this returns true, so code that waits
	 * for the flywheel (the autos' launch step) moves on instead of waiting forever.
	 */
	public boolean isAtSpeed() {
		if (!installed) {
			return true;
		}

		if (state != LauncherState.ACTIVE) {
			return false;
		}

		double targetRPM = ticksPerSecToRPM(targetTPS);

		// Check both motors separately - both must be within tolerance
		double rightError = Math.abs(targetRPM - averagedRightRPM);
		double leftError = Math.abs(targetRPM - averagedLeftRPM);

		return rightError < MAX_SPEED_ERROR && leftError < MAX_SPEED_ERROR;
	}

	public enum LauncherState {
		IDLE,
		ACTIVE
	}
}