package org.firstinspires.ftc.teamcode;

/** Motor encoder conversion for the goBILDA 5203 Yellow Jacket 6000 RPM motor, 1:1 to flywheel. */
public final class ShooterSpeed {
    private ShooterSpeed() { }
    public static final double MOTOR_TICKS_PER_REV = BareMotorSpecifications.ENCODER_TICKS_PER_REV;
    public static final double MOTOR_MAX_RPM = 6000.0;
    // Team observation, November 20, 2026: full power, observed maximum 6100 RPM at 14 V
    // (supersedes the November 1 report of 5400 RPM at 14 V).
    // Installed-speed estimate used for feedforward and kV, not a command cap.
    public static final double REPORTED_MAX_OBSERVED_RPM = 6100.0;
    public static final double REPORTED_MEASUREMENT_VOLTAGE_VOLTS = 14.0;
    public static final double FLYWHEEL_REVS_PER_MOTOR_REV = 1.0;
    public static final double FLYWHEEL_MAX_RPM = MOTOR_MAX_RPM * FLYWHEEL_REVS_PER_MOTOR_REV;
    // Operator-requested full target range; no software recovery reserve.
    public static final double FLYWHEEL_MAX_TARGET_RPM = FLYWHEEL_MAX_RPM;

    public static double clampFlywheelRpm(double rpm) {
        return Double.isFinite(rpm) ? Math.max(-FLYWHEEL_MAX_TARGET_RPM, Math.min(FLYWHEEL_MAX_TARGET_RPM, rpm)) : 0;
    }
    public static double motorTicksPerSecond(double flywheelRpm) {
        return clampFlywheelRpm(flywheelRpm) / FLYWHEEL_REVS_PER_MOTOR_REV / 60 * MOTOR_TICKS_PER_REV;
    }
    public static double flywheelRpm(double motorTicksPerSecond) {
        return motorTicksPerSecond * 60 / MOTOR_TICKS_PER_REV * FLYWHEEL_REVS_PER_MOTOR_REV;
    }
}
