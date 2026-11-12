package org.firstinspires.ftc.teamcode;

/** Initial hub velocity PIDF gains, voltage-compensated feedforward and the bench-only
 * bang-bang + feedforward fusion law; all require loaded tuning. */
public final class ShooterVelocityTuning {
    private ShooterVelocityTuning() { }
    // Stronger initial proportional recovery; validate overshoot during loaded trials.
    public static final double INITIAL_P = 40.0;
    public static final double INITIAL_I = 0.0;
    public static final double INITIAL_D = 0.0;
    public static final double INITIAL_F_SCALE = 1.0;
    public static final double MAX_I = 5.0;
    public static final double I_STEP = .05;
    public static final double MIN_F_CHANGE = .02;
    public static final double SAMPLE_INTERVAL_SECONDS = .2;
    // Lynx velocity feedforward output is scaled to +/-32767.
    // Initial installed-speed fit: 5400 RPM at 14 V. Normalize BASE_F to 12 V;
    // this assumes linear voltage scaling and still requires loaded validation.
    public static final double FULL_SCALE_OUTPUT = 32767.0;
    public static final double BASE_F = FULL_SCALE_OUTPUT
            / (ShooterSpeed.REPORTED_MAX_OBSERVED_RPM / 60 * ShooterSpeed.MOTOR_TICKS_PER_REV)
            * ShooterSpeed.REPORTED_MEASUREMENT_VOLTAGE_VOLTS
            / BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS;

    // ---- Bang-bang + feedforward fusion (bench comparison controller; untuned initial values) ----
    /** Above this absolute speed error the fusion controller is bang-bang: full power when slow, coast when fast. */
    public static final double FUSION_BANG_BANG_ERROR_TICKS_PER_SECOND = 80.0;
    // Motor power (0..1) at the 12 V reference; all three are estimates until measured on the robot.
    // kS: power that just overcomes static friction. Estimate, not a measurement.
    public static final double INITIAL_FUSION_KS = 0.05;
    // kV: power per motor tick/s. Solved so kS + kV * speed reproduces the installed-speed
    // observation (full power = 5400 RPM at 14 V) after voltage compensation.
    public static final double INITIAL_FUSION_KV = (ShooterSpeed.REPORTED_MEASUREMENT_VOLTAGE_VOLTS
            / BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS - INITIAL_FUSION_KS)
            / (ShooterSpeed.REPORTED_MAX_OBSERVED_RPM / 60 * ShooterSpeed.MOTOR_TICKS_PER_REV);
    // kP: small correction, power per tick/s of remaining error (0.08 power at the 80 ticks/s edge).
    public static final double INITIAL_FUSION_KP = 0.001;
    public static final double MAX_FUSION_KS = 0.3;
    public static final double MAX_FUSION_KV = 0.002;
    public static final double MAX_FUSION_KP = 0.02;

    /** True when the speed error is large enough that the fusion controller is in its bang-bang zone. */
    public static boolean fusionUsesBangBang(double targetTicksPerSecond, double measuredTicksPerSecond) {
        return Double.isFinite(targetTicksPerSecond) && Double.isFinite(measuredTicksPerSecond)
                && targetTicksPerSecond != 0 && Math.abs(targetTicksPerSecond - measuredTicksPerSecond)
                > FUSION_BANG_BANG_ERROR_TICKS_PER_SECOND;
    }

    /**
     * Fusion flywheel power. Speed error above 80 ticks/s: bang-bang, full power when slow and
     * zero power (FLOAT coast) when fast. Inside that band: kS + kV * target + kP * error, scaled
     * by 12 V / battery so the same gains hold as the battery sags. The command never opposes the
     * target direction, so the flywheel is never reverse-power braked. A zero target or any
     * non-finite input commands zero. Supports either commanded direction.
     */
    public static double fusionPower(double targetTicksPerSecond, double measuredTicksPerSecond,
            double batteryVolts, double kS, double kV, double kP) {
        if (!RobotMath.finite(targetTicksPerSecond, measuredTicksPerSecond, kS, kV, kP)
                || targetTicksPerSecond == 0) return 0;
        double direction = Math.signum(targetTicksPerSecond);
        // Positive when the wheel is slower than commanded, in the commanded direction.
        double error = (targetTicksPerSecond - measuredTicksPerSecond) * direction;
        if (error > FUSION_BANG_BANG_ERROR_TICKS_PER_SECOND) return direction;
        if (error < -FUSION_BANG_BANG_ERROR_TICKS_PER_SECOND) return 0;
        double voltage = Double.isFinite(batteryVolts) && batteryVolts >= 9 && batteryVolts <= 16
                ? batteryVolts : BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS;
        double power = (kS + kV * Math.abs(targetTicksPerSecond) + kP * error)
                * BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS / voltage;
        return direction * Math.max(0, Math.min(1, power));
    }

    public static double feedforward(double batteryVolts, double scale) {
        double voltage = Double.isFinite(batteryVolts) && batteryVolts >= 9 && batteryVolts <= 16
                ? batteryVolts : BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS;
        double boundedScale = Double.isFinite(scale) ? Math.max(.5, Math.min(1.5, scale)) : 1;
        return BASE_F * BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS / voltage * boundedScale;
    }
    public static double estimatedFreeSpeedUsage(double outputRpm, double batteryVolts) {
        double volts = Double.isFinite(batteryVolts) && batteryVolts >= 9 && batteryVolts <= 16
                ? batteryVolts : BareMotorSpecifications.REFERENCE_VOLTAGE_VOLTS;
        return Math.abs(outputRpm) / ShooterSpeed.REPORTED_MAX_OBSERVED_RPM
                * ShooterSpeed.REPORTED_MEASUREMENT_VOLTAGE_VOLTS / volts;
    }
}
