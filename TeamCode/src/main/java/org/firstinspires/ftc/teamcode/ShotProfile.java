package org.firstinspires.ftc.teamcode;

/** Measured settings: RPM is flywheel output RPM; hoodDeg is 0..30 deg downward travel from home. */
public final class ShotProfile {
    public static final class Row {
        public final double distance, hoodDeg, rpm;
        public Row(double distance, double hoodDeg, double rpm) {
            this.distance = distance; this.hoodDeg = hoodDeg; this.rpm = rpm;
        }
    }
    public final String geometryId;
    public final boolean calibrated;
    /** True: ranges outside the rows continue the nearest end segment's line (clamped to mechanism limits). */
    public final boolean extrapolates;
    private final Row[] rows;
    public ShotProfile(String geometryId, boolean calibrated, Row... rows) {
        this(geometryId, calibrated, false, rows);
    }
    public ShotProfile(String geometryId, boolean calibrated, boolean extrapolates, Row... rows) {
        this.extrapolates = extrapolates;
        if (geometryId == null || geometryId.isEmpty() || rows == null || rows.length < 2)
            throw new IllegalArgumentException("Profile needs geometry and at least two rows");
        this.geometryId = geometryId; this.calibrated = calibrated; this.rows = rows.clone();
        double previous = 0;
        for (Row row : this.rows) {
            if (row == null || !RobotMath.finite(row.distance, row.hoodDeg, row.rpm)
                    || row.distance <= previous || row.hoodDeg < TurretSubsystem.PITCH_MIN_DEG
                    || row.hoodDeg > TurretSubsystem.PITCH_MAX_DEG
                    || row.rpm <= 0 || row.rpm > ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM)
                throw new IllegalArgumentException("Invalid profile row / coverage ordering");
            previous = row.distance;
        }
    }
    public Row sample(double distance) {
        if (!Double.isFinite(distance)) return null;
        Row first = rows[0], last = rows[rows.length - 1];
        if (distance < first.distance || distance > last.distance) {
            if (!extrapolates || distance < 0) return null;
            Row a = distance < first.distance ? first : rows[rows.length - 2];
            Row b = distance < first.distance ? rows[1] : last;
            Row end = distance < first.distance ? first : last;
            double t = (distance - a.distance) / (b.distance - a.distance);
            double hood = clamp(a.hoodDeg + t * (b.hoodDeg - a.hoodDeg),
                    TurretSubsystem.PITCH_MIN_DEG, TurretSubsystem.PITCH_MAX_DEG);
            // Floor keeps a falling near-range line from commanding a stalled flywheel.
            double rpm = clamp(a.rpm + t * (b.rpm - a.rpm),
                    Math.min(TurretSubsystem.SHOOTER_MIN_AUTO_RPM, end.rpm), ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM);
            return new Row(distance, hood, rpm);
        }
        for (int i = 1; i < rows.length; i++) {
            Row a = rows[i - 1], b = rows[i];
            if (distance <= b.distance) {
                double t = (distance - a.distance) / (b.distance - a.distance);
                return new Row(distance, a.hoodDeg + t * (b.hoodDeg - a.hoodDeg),
                        a.rpm + t * (b.rpm - a.rpm));
            }
        }
        return null;
    }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    public static ShotProfile example() {
        // Keep unmeasured examples below the configured target ceiling as the motor baseline changes.
        double ratio = ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM / BareMotorSpecifications.NOMINAL_RPM;
        return new ShotProfile("UNMEASURED", false,
                new Row(.6096, 2, 3200 * ratio), new Row(.9144, 4.5, 3550 * ratio),
                new Row(1.2192, 7, 3900 * ratio), new Row(1.524, 9.5, 4250 * ratio),
                new Row(1.8288, 11.5, 4650 * ratio), new Row(2.1336, 13.5, 5050 * ratio),
                new Row(2.4384, 15, 5450 * ratio));
    }
}
