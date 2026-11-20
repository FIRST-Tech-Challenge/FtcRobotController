package org.firstinspires.ftc.teamcode;

/** Operator offsets to a calibrated solution, retained until explicitly cleared or rehomed. */
public final class AutoAimAdjustments {
    public static final double MAX_YAW_DEG = 10, MAX_HOOD_DEG = 5, MAX_RPM = 500;
    public static final double YAW_RATE_DEG_PER_SECOND = 15;
    public static final double HOOD_STEP_DEG = .5, RPM_STEP = 50;
    private double yawDeg, hoodDeg, rpm;

    public void adjust(double clockwiseStick, double dt, int hoodSteps, int rpmSteps) {
        if (Math.abs(clockwiseStick) > TurretControl.MANUAL_STICK_DEADBAND)
            yawDeg = clip(yawDeg - clockwiseStick * Math.max(0, Math.min(.1, dt))
                    * YAW_RATE_DEG_PER_SECOND, MAX_YAW_DEG);
        hoodDeg = clip(hoodDeg + hoodSteps * HOOD_STEP_DEG, MAX_HOOD_DEG);
        rpm = clip(rpm + rpmSteps * RPM_STEP, MAX_RPM);
    }
    /** Absolute trims (planner volley parameters), clipped to the same bounds as the operator buttons. */
    public void set(double yawDeg, double hoodDeg, double rpm) {
        this.yawDeg = clip(finite(yawDeg), MAX_YAW_DEG);
        this.hoodDeg = clip(finite(hoodDeg), MAX_HOOD_DEG);
        this.rpm = clip(finite(rpm), MAX_RPM);
    }
    private static double finite(double value) { return Double.isFinite(value) ? value : 0; }
    private static double clip(double value, double max) { return Math.max(-max, Math.min(max, value)); }
    public void reset() { yawDeg = hoodDeg = rpm = 0; }
    public double yawDeg() { return yawDeg; }
    public double hoodDeg() { return hoodDeg; }
    public double rpm() { return rpm; }
}
