package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    /** Calibrated at 12.76 V; command is compensated for measured battery voltage. */
    public static double INTAKE_POWER = 0.85;
    public static double REFERENCE_VOLTAGE = 12.76;

    // Verify on the robot using Ramp Tuning, then copy the measured values here.
    public static final double RAMP_DOWN = 0.1;
    public static final double RAMP_UP = 0.62;
    public static final double RAMP_HOLD = 0.15;
    public static final double RAMP_DOWNER = 0.23;

    public enum RampState { UP, DOWN, HOLD, DOWNER }

    private final DcMotorEx intake;
    private final Servo ramp;
    private final BatteryVoltage battery;
    private RampState rampState = RampState.UP;
    private boolean intakeOn = false;
    private double requestedPower;

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, new BatteryVoltage(hardwareMap));
    }

    /** Preferred: share one BatteryVoltage between Launcher, Transfer and Intake. */
    public Intake(HardwareMap hardwareMap, BatteryVoltage battery) {
        this(hardwareMap.get(DcMotorEx.class, "Intake"),
                hardwareMap.get(Servo.class, "Ramp"), battery);
    }

    // Hardware injection for desktop regression tests.
    Intake(DcMotorEx intake, Servo ramp) {
        this(intake, ramp, null);
    }

    Intake(DcMotorEx intake, Servo ramp, BatteryVoltage battery) {
        this.intake = intake;
        this.ramp = ramp;
        this.battery = battery;
        stop();
        // Intake and outtake are power commands; stop must not switch to velocity control.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setRampState(RampState.UP);
    }

    public void intake() {
        setPower(INTAKE_POWER);
    }

    /** Sets a requested normalized power and applies battery compensation. */
    public void setPower(double power) {
        requestedPower = Math.max(-1.0, Math.min(1.0, power));
        intakeOn = Math.abs(requestedPower) > 0;
        if (!intakeOn) intake.setPower(0.0);
        else update();
    }

    public void toggle() {
        if (intakeOn) {
            stop();
        }
        else {
            intake();
        }

    }

    public void outtake() {
        setPower(-INTAKE_POWER);
    }

    public void stop() {
        requestedPower = 0.0;
        intake.setPower(0.0);
        intakeOn = false;
    }

    /** Call every OpMode loop so compensation tracks battery sag during operation. */
    public void update() {
        if (!intakeOn) return;
        double voltage = readBatteryVoltage();
        double compensated = voltage > 0 && Double.isFinite(REFERENCE_VOLTAGE)
                ? requestedPower * REFERENCE_VOLTAGE / voltage : requestedPower;
        intake.setPower(Math.max(-1.0, Math.min(1.0, compensated)));
    }

    private double readBatteryVoltage() {
        if (battery == null) return Double.NaN;
        double voltage = battery.get();
        return voltage >= 6.0 && voltage <= 16.8 ? voltage : Double.NaN;
    }

    /** UP toggles to DOWN; DOWN, HOLD and DOWNER toggle to UP. */
    public void toggleRamp() {
        setRampState(rampState == RampState.UP ? RampState.DOWN : RampState.UP);
    }

    public void rampDowner() { setRampState(RampState.DOWNER); }
    public void holdBalls() { setRampState(RampState.HOLD); }

    private void setRampState(RampState state) {
        ramp.setPosition(rampPositionFor(state));
        rampState = state;
    }

    public static double rampPositionFor(RampState state) {
        switch (state) {
            case UP: return RAMP_UP;
            case DOWN: return RAMP_DOWN;
            case HOLD: return RAMP_HOLD;
            case DOWNER: return RAMP_DOWNER;
            default: throw new IllegalArgumentException("Unknown ramp state");
        }
    }

    public RampState getRampState() { return rampState; }
    public boolean isRampUp() { return rampState == RampState.UP; }
    public boolean isHoldingBalls() { return rampState == RampState.HOLD; }
    public double rampthingy() { return ramp.getPosition(); }
    public double getIntakeVelo() { return intake.getVelocity(); }
    public boolean isIntakeOn() { return intakeOn; }
}
