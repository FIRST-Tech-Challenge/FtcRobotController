package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.bylazar.configurables.annotations.Configurable;

/** Single-motor, 1:1 shooter using only FTC SDK DcMotorEx. */
@Configurable
public class ShooterSubsystemFF {
    public static double TICKS_PER_REV = 28.0; // Check your motor's encoder spec
    public static double MAX_RPM = 6000.0;
    public static double READY_TOLERANCE_RPM = 150.0;

    // Initial estimates only; tune on your actual robot.
    public static double kP = 0.001; // power per RPM of error
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0002; // power per target RPM
    public static double kS = 0.04;

    private final DcMotorEx motor;
    public static double targetRpm = 3200.0;
    private double actualRpm = 0.0;
    private double power = 0.0;
    private double integral = 0.0;
    private double previousError = 0.0;
    private long previousTimeNs = 0;
    private boolean on = false;

    public ShooterSubsystemFF(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor_one");
        motor.setDirection(DcMotor.Direction.REVERSE); // Change if rotation is wrong
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0.0);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Range.clip(rpm, 0.0, MAX_RPM);
    }

    /** Call once per TeleOp loop. */
    public void update(boolean enabled) {
        on = enabled && targetRpm > 0.0;
        if (!on) {
            motor.setPower(0.0);
            power = 0.0;
            actualRpm = 0.0; // No new reading while off
            integral = 0.0;
            previousError = 0.0;
            previousTimeNs = 0;
            return;
        }

        actualRpm = motor.getVelocity() * 60.0 / TICKS_PER_REV; // 1:1 drive
        double error = targetRpm - actualRpm;
        long now = System.nanoTime();
        double dt = previousTimeNs == 0 ? 0.0 : (now - previousTimeNs) / 1e9;
        previousTimeNs = now;

        double derivative = 0.0;
        if (dt > 0.0 && dt < 0.25) {
            // Avoid building up integral error while the output is saturated.
            if (Math.abs(power) < 0.999 || (power >= 0.999 && error < 0)
                    || (power <= 0.001 && error > 0)) {
                integral += error * dt;
            }
            derivative = (error - previousError) / dt;
        }
        previousError = error;

        double feedforward = kV * targetRpm + kS;
        power = Range.clip(feedforward + kP * error + kI * integral + kD * derivative,
                0.0, 1.0);
        motor.setPower(power);
    }

    public boolean isReady() {
        return on && Math.abs(targetRpm - actualRpm) <= READY_TOLERANCE_RPM;
    }

    public double getTargetRpm() { return targetRpm; }
    public double getVelocityRpm() { return actualRpm; }
    public double getPowerCmd() { return power; }
    public boolean isOn() { return on; }

    public void stop() { update(false); }
}
