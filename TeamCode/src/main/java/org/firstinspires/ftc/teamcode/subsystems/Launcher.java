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
 * The flywheel holds a target velocity (ticks/sec) using the motor controller's PIDF.
 * Teleop uses KP..KF and TARGET_VELOCITY, autos use the AUTO_ values. All are in LauncherConfig.
 *
 *  - useAutoValues()
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
    private boolean autoValues = false;

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
        // Every loop, so Dashboard changes to PIDF or velocity take effect right away.
        applyPidf();
        if (spinning) flywheel.setVelocity(targetVelocity());

        double target = spinning ? targetVelocity() : 0;
        telemetry.addData("Flywheel vel (ticks/s)", "%.0f", getVelocity());
        telemetry.addData("Flywheel target", "%.0f", target);
        telemetry.addData("Flywheel error", "%.0f", target - getVelocity());
        telemetry.addData("Flywheel ready", isReady());
        telemetry.addData("Flywheel values", autoValues ? "auto" : "teleop");
    }

    @Override
    public void stop() {
        stopFlywheel();
        retractFeeder();
    }

    /** Autos call this once so the flywheel uses the AUTO_ values in LauncherConfig. */
    public void useAutoValues() {
        autoValues = true;
        applyPidf();
    }

    /** Spin the flywheel to the target velocity and keep it there. */
    public void spinUp() {
        spinning = true;
        flywheel.setVelocity(targetVelocity());
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

    /** Flywheel speed in ticks/sec. */
    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public boolean isReady() {
        return spinning
                && Math.abs(getVelocity() - targetVelocity()) < LauncherConfig.VELOCITY_TOLERANCE;
    }

    public void feed() {
        feeder.setPosition(LauncherConfig.FEEDER_FEED_POS);
    }

    public void retractFeeder() {
        feeder.setPosition(LauncherConfig.FEEDER_REST_POS);
    }

    private double targetVelocity() {
        return autoValues ? LauncherConfig.AUTO_TARGET_VELOCITY : LauncherConfig.TARGET_VELOCITY;
    }

    private void applyPidf() {
        if (autoValues) {
            flywheel.setVelocityPIDFCoefficients(LauncherConfig.AUTO_KP, LauncherConfig.AUTO_KI,
                    LauncherConfig.AUTO_KD, LauncherConfig.AUTO_KF);
        } else {
            flywheel.setVelocityPIDFCoefficients(LauncherConfig.KP, LauncherConfig.KI,
                    LauncherConfig.KD, LauncherConfig.KF);
        }
    }
}
