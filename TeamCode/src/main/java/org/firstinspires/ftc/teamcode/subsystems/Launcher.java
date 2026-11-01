package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * LAUNCHER - flywheel plus the feeder that pushes a piece into it.
 *
 * How it works: the flywheel motor holds a target speed (RPM) using the motor
 * controller's built-in PIDF. Tune KP/KI/KD/KF in LauncherConfig (live from FTC Dashboard).
 *
 * Put here:
 *  - spinUp(rpm), stopFlywheel(), reverse(power) - run backward to clear a jam
 *  - isReady() - true when the flywheel is close enough to the target speed
 *  - feed() / retractFeeder()
 * Later: pick the RPM from the distance to the goal.
 */
public class Launcher implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private DcMotorEx flywheel;
    private Servo feeder;

    private double targetVel = 0;

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
        applyPidf(); // every loop, so Dashboard changes take effect
        telemetry.addData("Launcher RPM", getRpm());
        telemetry.addData("Launcher target", targetVel);
    }

    @Override
    public void stop() {
        stopFlywheel();
        retractFeeder();
    }

    public void spinUp(double vel) {
        if (vel != targetVel) RobotLog.ii("Launcher", "target rpm -> %.0f", vel);
        targetVel = vel;
        flywheel.setVelocity(vel * RobotConstants.LAUNCHER_TICKS_PER_REV / 60.0);
    }

    public void stopFlywheel() {
        targetVel = 0;
        flywheel.setPower(0);
    }

    /** Runs the flywheel backward at raw power (not velocity-controlled) to clear a stuck ball. */
    public void reverse(double power) {
        targetVel = 0; // not holding a shooting speed while reversing
        flywheel.setPower(-Math.abs(power));
    }

    public double getRpm() {
        return flywheel.getVelocity() * 60.0 / RobotConstants.LAUNCHER_TICKS_PER_REV;
    }

    public boolean isReady() {
        return targetVel > 0
                && Math.abs(getRpm() - targetVel) < LauncherConfig.RPM_TOLERANCE;
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
