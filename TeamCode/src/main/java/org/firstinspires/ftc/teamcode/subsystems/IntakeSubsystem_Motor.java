package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeSubsystem_Motor {

    // Assumes both motors use 28 encoder ticks per revolution.
    private static final double TICKS_PER_REV = 28.0;

    public static double NORMAL_RPM = 6000.0;
    public static double SLOW_RPM = 3000.0;

    private final DcMotorEx motor;
    private final DcMotorEx motor2;

    public boolean isOn = false;
    private double targetRpm = 0.0;

    public IntakeSubsystem_Motor(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "intakeMotor2");

        configureMotor(motor);
        configureMotor(motor2);

        off();
    }

    private void configureMotor(DcMotorEx intakeMotor) {
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private void commandRpm(double rpm) {
        targetRpm = rpm;
        isOn = rpm != 0.0;

        double ticksPerSecond = rpmToTicksPerSecond(rpm);

        motor.setVelocity(ticksPerSecond);
        motor2.setVelocity(ticksPerSecond);
    }

    // Run both motors at NORMAL_RPM.
    public void on() {
        commandRpm(NORMAL_RPM);
    }

    // Mode 1 uses SLOW_RPM; other values use NORMAL_RPM.
    public void on(int mode) {
        commandRpm(mode == 1 ? SLOW_RPM : NORMAL_RPM);
    }

    public void off() {
        commandRpm(0.0);
    }

    public void startIntake() {
        on();
    }

    public void startIntake(int mode) {
        on(mode);
    }

    public void stopIntake() {
        off();
    }

    // Reverse both motors at the current target speed.
    // If stopped, use NORMAL_RPM.
    public void reverseIntake() {
        double rpm = isOn ? Math.abs(targetRpm) : NORMAL_RPM;
        commandRpm(-rpm);
    }

    // Actual RPM of the first motor.
    public double getVelocityRpm() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    // Actual RPM of the second motor.
    public double getVelocityRpm2() {
        return motor2.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public double getTargetRpm() {
        return targetRpm;
    }
}