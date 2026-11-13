package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Spins the launcher flywheel at a target velocity and tells you when it is up to speed.
 *
 * The motor's built-in velocity control (RUN_USING_ENCODER + PIDF) does the work: once a target
 * is set, the motor keeps holding it on its own. Velocities are in encoder ticks per second.
 *
 * Typical use:
 * <pre>
 *     launcher.setTargetVelocity(1250);      // start spinning up
 *     ...
 *     if (launcher.isAtSpeed()) feed();      // only shoot once it's up to speed
 *     ...
 *     launcher.stop();
 * </pre>
 */
public class Launcher {

    // ---------------- Tuning / configuration ----------------
    public static double MIN_TARGET_VELOCITY = 200;   // ticks/s
    public static double MAX_TARGET_VELOCITY = 2000;  // ticks/s
    public static double AT_SPEED_TOLERANCE = 50;     // "at speed" if within this many ticks/s

    // Velocity PIDF for the motor controller. Adjust these for spinup and recovery.
    public static double KP = 40;
    public static double KI = 2;
    public static double KD = 0;
    public static double KF = 14.65;

    // ---------------- Hardware and state ----------------
    private final DcMotorEx motor;
    private final Telemetry telemetry;

    private double targetVelocity = 0;  // 0 = stopped

    public Launcher(DcMotorEx motor, Telemetry telemetry) {
        this.motor = motor;
        this.telemetry = telemetry;

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(KP, KI, KD, KF)
        );
    }

    // ---------------- Control ----------------

    /**
     * Spin at this velocity, kept between MIN_TARGET_VELOCITY and MAX_TARGET_VELOCITY.
     * 0 or less stops the launcher.
     */
    public void setTargetVelocity(double velocity) {
        if (velocity <= 0) {
            stop();
            return;
        }
        targetVelocity = Range.clip(velocity, MIN_TARGET_VELOCITY, MAX_TARGET_VELOCITY);
        motor.setVelocity(targetVelocity);
    }

    /** Raise (positive) or lower (negative) the target velocity, e.g. from a D-pad press. */
    public void changeTargetVelocity(double change) {
        setTargetVelocity(targetVelocity + change);
    }

    /** Stop driving the flywheel and let it spin down. */
    public void stop() {
        targetVelocity = 0;
        motor.setPower(0);
    }

    // ---------------- What the launcher knows ----------------

    /** True if the launcher has a target velocity (it was started and not stopped). */
    public boolean isRunning() {
        return targetVelocity > 0;
    }

    /** True if the launcher is running and within AT_SPEED_TOLERANCE of its target. */
    public boolean isAtSpeed() {
        return isRunning() && Math.abs(getVelocity() - targetVelocity) < AT_SPEED_TOLERANCE;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** The flywheel's actual velocity right now, in ticks/s. */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** Add the launcher's state to telemetry. The caller still calls telemetry.update(). */
    public void showStatus() {
        String state;
        if (!isRunning()) state = "OFF";
        else if (isAtSpeed()) state = "AT SPEED";
        else state = "spinning up";
        telemetry.addData("Launcher", "%s   (%.0f / %.0f ticks/s)",
                state, getVelocity(), targetVelocity);
    }
}
