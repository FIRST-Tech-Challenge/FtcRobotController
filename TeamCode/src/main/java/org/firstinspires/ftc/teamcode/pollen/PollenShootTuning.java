package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.List;

/** INIT-only selections for the end-wall, forward-facing shooting start. No hardware calls. */
public final class PollenShootTuning {
    public enum Field {
        FLYWHEEL("Flywheel ticks/s", 500, 2500, 50, 1250),
        FEED_SECONDS("Feed seconds (0 = skip shot)", 0, 20, .5, 10),
        DISTANCE_CM("Travel cm (0 = shoot only)", 0, 350, 5, 60.96),
        DRIVE_POWER("Drive vector power", .10, .50, .05, .20),
        TRAVEL_ANGLE("Travel angle deg (+left)", -90, 90, 1, PARK_ANGLE_DEGREES),
        FEED_POWER("Intake + agitator power", .10, 1, .05, .50),
        START_DELAY("Start delay seconds", 0, 20, .5, 0);

        public final String label;
        private final double min, max, step, initial;
        Field(String label, double min, double max, double step, double initial) {
            this.label = label;
            this.min = min;
            this.max = max;
            this.step = step;
            this.initial = initial;
        }
    }

    // Forward-only experiment: distance_cm = seconds * power * 150 at 12.86 V.
    // Diagonal follow-up: 100 cm selected with that model produced 86 cm total,
    // including 76 cm forward. Updated diagonal factor = 150 * 86 / 100 = 129.
    // This ratio does not require assuming the test's power. Its voltage was not
    // reported. Provisional single-run fit: repeat at the intended power/load.
    // The observed direction was about 27.9 degrees left if 86 cm is straight-line
    // displacement. Retain 25.87 degrees as the default; INIT can select another angle.
    // Factor 129 was fitted near that angle, not across the full adjustment range.
    // Option A's forward-only settings and calibration are unchanged.
    public static final double CM_PER_SECOND_PER_POWER = 129;

    // Provisional geometry from BIOBUZZ manual TU05, figures 9-2 and 9-10:
    // 144-inch field; hive axes 25.5 inches apart. Assume an 18x18-inch robot,
    // centered on its hive axis and square to the end wall, shooting FORWARD.
    // Red center: (59.25,135) -> (11,35.5) inches, x right / y down on figure 9-2.
    // Blue is the 180-degree rotation: (84.75,9) -> (133,108.5).
    // Nominal FULL-FIELD distance = hypot(48.25,99.5)*2.54 = 280.88 cm (9.22 ft).
    // Nominal travel angle = 25.87 degrees LEFT of robot forward, WITHOUT turning.
    // This is NOT a validated route: measure the robot footprint, starting center,
    // loading-zone tape and especially swept clearance past the hive frame/feet.
    // If clearance fails, a waypoint route is needed; more distance cannot fix it.
    // Keep the 60.96-cm home-test default above until the complete route is checked.
    public static final double PARK_ANGLE_DEGREES = 25.87;
    public static final double NOMINAL_FIELD_DISTANCE_CM = 280.88;
    public static final double SPINUP_TIMEOUT = 4, SPEED_STABLE_SECONDS = .25;
    public static final double READY_FRACTION = .96, HOLD_SECONDS = .5, DEADLINE = 27;

    /** Process-local memory, not saved across a Robot Controller app restart. */
    public static final class Snapshot {
        private final double[] values;
        private Snapshot(double[] values) {
            this.values = values.clone();
        }
    }

    private double[] values = new double[Field.values().length];
    private int selected;
    private boolean locked;
    private final boolean[] previous = new boolean[5];
    private String notice = "";

    public PollenShootTuning(Snapshot remembered) {
        reset();
        if (remembered != null) {
            values = remembered.values.clone();
            values[Field.START_DELAY.ordinal()] = 0;
        }
    }

    private void reset() {
        for (Field f : Field.values()) values[f.ordinal()] = f.initial;
        notice = "Defaults restored: 2-foot home test";
    }

    /** D-pad selects/edits; triangle resets. Tap/release. */
    public void update(boolean up, boolean down, boolean left, boolean right,
            boolean reset) {
        if (locked) return;
        boolean[] keys = {up, down, left, right, reset};
        if (reset) {
            if (!previous[4]) reset();
        } else {
            if (up && !down && !previous[0]) selected = (selected + Field.values().length - 1) % Field.values().length;
            if (down && !up && !previous[1]) selected = (selected + 1) % Field.values().length;
            if (left && !right && !previous[2]) adjust(-1);
            if (right && !left && !previous[3]) adjust(1);
        }
        System.arraycopy(keys, 0, previous, 0, keys.length);
    }

    private void adjust(int sign) {
        Field f = selected();
        double old = value(f);
        // Preserve the tested fractional default; first angle edit reaches the
        // next whole degree in the requested direction, then steps by one degree.
        // This lets the operator select exactly zero without extra controls.
        double next = f == Field.TRAVEL_ANGLE
                ? (sign > 0 ? Math.floor(old) + 1 : Math.ceil(old) - 1)
                : Math.round((old + sign * f.step) * 100.0) / 100.0;
        values[selected] = Math.max(f.min, Math.min(f.max, next));
        if (worstCaseSeconds() >= DEADLINE - 1) {
            values[selected] = old;
            notice = "Change rejected: needs >=26s. Reduce feed/delay/distance or increase drive power.";
        } else notice = "";
    }

    public Field selected() { return Field.values()[selected]; }
    public double value(Field f) { return values[f.ordinal()]; }
    public String notice() { return notice; }
    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public Snapshot snapshot() { return new Snapshot(values); }

    public double driveSeconds() {
        return value(Field.DISTANCE_CM) / (value(Field.DRIVE_POWER) * CM_PER_SECOND_PER_POWER);
    }

    public double worstCaseSeconds() {
        return value(Field.START_DELAY) + driveSeconds() + HOLD_SECONDS
                + (value(Field.FEED_SECONDS) > 0 ? SPINUP_TIMEOUT + value(Field.FEED_SECONDS) : 0);
    }

    // Both alliance starts are related by a 180-degree rotation. Humans position
    // the robot facing its hive; the same robot-relative vector works on either side.
    public double forward() {
        return value(Field.DRIVE_POWER) * Math.cos(Math.toRadians(value(Field.TRAVEL_ANGLE)));
    }
    public double strafe() {
        return -value(Field.DRIVE_POWER) * Math.sin(Math.toRadians(value(Field.TRAVEL_ANGLE)));
    }

    public PollenSequence sequence() {
        return sequence(true);
    }

    /** Shooter-init fallback follows THIS starting pose's departure, never Park A's route. */
    public PollenSequence sequence(boolean shooterAvailable) {
        List<PollenSequence.Step> route = new ArrayList<>();
        if (shooterAvailable && value(Field.FEED_SECONDS) > 0) route.add(PollenSequence.Step.continuousShoot(value(Field.FEED_SECONDS)));
        if (value(Field.DISTANCE_CM) > 0) route.add(PollenSequence.Step.drive("DIAGONAL TO PARK", driveSeconds(), forward(), strafe(), 0));
        route.add(PollenSequence.Step.drive("PARK / HOLD", HOLD_SECONDS, 0, 0, 0));
        return new PollenSequence(route.toArray(new PollenSequence.Step[0]), value(Field.FLYWHEEL),
                value(Field.FLYWHEEL) * READY_FRACTION, SPINUP_TIMEOUT, SPEED_STABLE_SECONDS,
                DEADLINE, value(Field.START_DELAY));
    }
}
