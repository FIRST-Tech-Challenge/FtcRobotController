package org.firstinspires.ftc.teamcode.pollen;

/** Edit here, rebuild, and redeploy. These values are NOT a calibrated field route. */
public final class PollenConfig {
    private PollenConfig() { }

    // Team's BioBuzzStarterbotOpMode mapping (not the download's underscore names).
    public static final String[] DRIVE_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};
    // PROVISIONAL: matches the new StarterBot TeleOp. Other repo classes disagree.
    public static final boolean[] DRIVE_REVERSED = {true, false, true, false};

    // Launcher values from goBILDA 3200-2627-0004 / BioBuzzStarterbotOpMode (MIT).
    // NOT a verified scoring distance for the team's robot.
    public static final double LAUNCHER_TARGET = 1250; // encoder ticks/second, not RPM
    public static final double LAUNCHER_MIN = 1200;
    public static final double SPINUP_TIMEOUT = 4.0;
    public static final double SPEED_STABLE_TIME = 0.25;
    public static final double ACTIVE_FEED_SECONDS = 2.0;
    public static final double MAX_FEED_WINDOW = 6.0; // includes recovery pauses; tune on robot
    public static final double START_DELAY_SECONDS = 0.0; // default; Park can override during INIT
    public static final double TOTAL_TIME_LIMIT = 27.0; // local guard, not a rule claim

    // Tune these on the actual robot; seconds are NOT a measured distance.
    // The two manually mirrored start poses use this same forward-only route.
    public static final double PARK_FORWARD_SECONDS = 0.75;
    public static final double PARK_FORWARD_POWER = 0.20;

    // Short, low-power EXAMPLES for initial testing. Positive = forward/right/clockwise.
    // Replace with the measured route from your chosen start and heading to the park zone.
    public static PollenSequence.Step[] parkRoute() {
        return new PollenSequence.Step[] {
                PollenSequence.Step.drive("FORWARD TO PARK", PARK_FORWARD_SECONDS, PARK_FORWARD_POWER, 0, 0),
                PollenSequence.Step.drive("PARK / HOLD", 0.5, 0, 0, 0)
        };
    }

    // This route starts from the same chosen start, but its parking leg starts at the
    // shooting position. Tune it independently; do not reuse the park-only timing blindly.
    public static PollenSequence.Step[] shootRoute() {
        return new PollenSequence.Step[] {
                PollenSequence.Step.drive("MOVE TO SHOT (example)", 0.75, 0.20, 0, 0),
                PollenSequence.Step.drive("SETTLE", 0.5, 0, 0, 0),
                PollenSequence.Step.shoot(ACTIVE_FEED_SECONDS, MAX_FEED_WINDOW),
                PollenSequence.Step.drive("MOVE TO PARK (example)", 0.75, -0.20, 0, 0),
                PollenSequence.Step.drive("PARK / HOLD", 0.5, 0, 0, 0)
        };
    }
}
