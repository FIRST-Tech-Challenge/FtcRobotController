package org.firstinspires.ftc.teamcode.pollen;

/** Shared drive mapping and Option A defaults. INIT controls allow tuning without rebuilding. */
public final class PollenConfig {
    private PollenConfig() { }

    // Team's BioBuzzStarterbotOpMode mapping (not the download's underscore names).
    public static final String[] DRIVE_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};
    // Matches the current PollenBot TeleOp and the tested autonomous drivetrain.
    public static final boolean[] DRIVE_REVERSED = {
            true, false, false, true
    };

    public static final double START_DELAY_SECONDS = 0.0; // default; Park can override during INIT
    public static final double TOTAL_TIME_LIMIT = 27.0; // local guard, not a rule claim

    // Option A uses timed forward movement; retain the tested defaults.
    // The two manually mirrored start poses use this same forward-only route.
    public static final double PARK_FORWARD_SECONDS = 2;
    public static final double PARK_FORWARD_POWER = 0.20;

    // Positive drive axes = forward/right/clockwise. Start pose is set by humans.
    public static PollenSequence.Step[] parkRoute() {
        return parkRoute(PARK_FORWARD_SECONDS, PARK_FORWARD_POWER);
    }

    public static final double PARK_HOLD_SECONDS = 0.5;

    /** Snapshot of the settings selected before START. */
    public static PollenSequence.Step[] parkRoute(double seconds, double power) {
        return new PollenSequence.Step[] {
                PollenSequence.Step.drive("FORWARD TO PARK", seconds, power, 0, 0),
                PollenSequence.Step.drive("PARK / HOLD", PARK_HOLD_SECONDS, 0, 0, 0)
        };
    }

}
