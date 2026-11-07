package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * LAUNCHER - flywheel plus the feeder that pushes a piece into it.
 *
 * The flywheel motor holds LauncherConfig.TARGET_VELOCITY (ticks/sec) using the motor
 * controller's built-in PIDF, exactly like SuperTestTeleop. Every value is in LauncherConfig
 * so it can be tuned live from FTC Dashboard.
 *
 *  - spinUp() / stopFlywheel() / reverse(power)
 *  - isReady() - flywheel within VELOCITY_TOLERANCE of the target
 *  - feed() / retractFeeder()
 */
public class Launcher implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private DcMotorEx flywheel;
    private Servo feeder;

    private boolean spinning = false;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, RobotConstants.LAUNCHER_MOTOR);
        feeder = hardwareMap.get(Servo.class, RobotConstants.FEEDER_SERVO);

        flywheel.setDirection(RobotConstants.LAUNCHER_MOTOR_DIRECTION);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPidf();

        retractFeeder();
    }

    @Override
    public void update() {
        // Every loop, so Dashboard changes to PIDF or TARGET_VELOCITY take effect right away.
        applyPidf();
        if (spinning) flywheel.setVelocity(LauncherConfig.TARGET_VELOCITY);

        double target = spinning ? LauncherConfig.TARGET_VELOCITY : 0;
        telemetry.addData("Flywheel vel (ticks/s)", "%.0f", getVelocity());
        telemetry.addData("Flywheel target", "%.0f", target);
        telemetry.addData("Flywheel error", "%.0f", target - getVelocity());
        telemetry.addData("Flywheel ready", isReady());
        telemetry.addData("PIDF", "%.1f / %.2f / %.1f / %.1f",
                LauncherConfig.KP, LauncherConfig.KI, LauncherConfig.KD, LauncherConfig.KF);
    }

    @Override
    public void stop() {
        stopFlywheel();
        retractFeeder();
    }

    /** Spin the flywheel to LauncherConfig.TARGET_VELOCITY and keep it there. */
    public void spinUp() {
        spinning = true;
        flywheel.setVelocity(LauncherConfig.TARGET_VELOCITY);
    }

    public void stopFlywheel() {
        spinning = false;
        flywheel.setPower(0);
    }

    /** Runs the flywheel backward to clear a stuck ball. */
    public void reverse(double power) {
        spinning = false;
        flywheel.setPower(-Math.abs(power));
    }

    /** Flywheel speed in ticks/sec (same units as TARGET_VELOCITY). */
    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public boolean isReady() {
        return spinning
                && Math.abs(getVelocity() - LauncherConfig.TARGET_VELOCITY) < LauncherConfig.VELOCITY_TOLERANCE;
    }

    public void feed() {
        feeder.setPosition(LauncherConfig.FEEDER_FEED_POS);
    }

    public void retractFeeder() {
        feeder.setPosition(LauncherConfig.FEEDER_REST_POS);
    }

    private void applyPidf() {
        flywheel.setVelocityPIDFCoefficients(
                LauncherConfig.KP, LauncherConfig.KI, LauncherConfig.KD, LauncherConfig.KF);
    }
}
