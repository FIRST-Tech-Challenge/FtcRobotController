package org.firstinspires.ftc.teamcode;

/**
 * Shared competition feed policy: CONTINUOUS only (2026-11-19). The staggered mode and its GP2 B toggle were
 * removed from competition TeleOp, so no staggered gate delay, timed feed window or recovery pause ever blocks
 * a competition shot. Once a feed is authorized, the gate-open command and the transfer run together.
 */
public final class CompetitionFeedController {
    // Shared timing values (see ShootingConfiguration). In continuous mode only READY_STABLE_SECONDS applies
    // (automatic shots only; manual feed bypasses readiness); the gate delay is skipped.
    public static final double GATE_TO_FEED_DELAY_SECONDS = ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS;
    public static final double FEED_WINDOW_SECONDS = ShootingConfiguration.FEED_WINDOW_SECONDS;
    public static final double RECOVERY_SECONDS = ShootingConfiguration.RECOVERY_SECONDS;
    public static final double READY_STABLE_SECONDS = ShootingConfiguration.READY_STABLE_SECONDS;
    private final StaggeredFeedTiming stage1Timing = new StaggeredFeedTiming();

    public static FiringCoordinator firing() {
        FiringCoordinator firing = new FiringCoordinator(GATE_TO_FEED_DELAY_SECONDS,
                ShootingConfiguration.FEED_TO_EXIT_SECONDS, FEED_WINDOW_SECONDS,
                RECOVERY_SECONDS, READY_STABLE_SECONDS);
        firing.setImmediateContinuousFeed(true);
        return firing;
    }

    /** Call before updating firing. Keeps the coordinator in continuous mode (configures it on the first call). */
    public void updateSelection(double now, FiringCoordinator firing) {
        if (firing.configureFeedMode(now, true, FEED_WINDOW_SECONDS,
                RECOVERY_SECONDS, READY_STABLE_SECONDS, true)) stage1Timing.reset();
    }

    /** Call after updating firing, including idle/inhibited loops. Stage 1 runs for the whole feed. */
    public boolean feedStage1(double now, FiringCoordinator firing) {
        return stage1Timing.feedStage1(now, firing);
    }

    /** Closed-gate stage-1 run between timed shots; never true in continuous mode. */
    public boolean closedGateStage1(FiringCoordinator firing) {
        return firing.closedGateStage1();
    }

    /** Continuous: the transfer feeds whenever firing is feeding, without waiting for the gate. */
    public boolean feedTransfers(FiringCoordinator firing, boolean gateReady) {
        return firing.feeding();
    }

    public boolean continuous() { return true; }
}
