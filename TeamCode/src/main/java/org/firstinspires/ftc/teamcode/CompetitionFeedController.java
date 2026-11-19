package org.firstinspires.ftc.teamcode;

/** Shared Red/Blue mode selection and non-blocking transfer feed timing. */
public final class CompetitionFeedController {
    // Same robot-tuned staggered timing as autonomous and the bench test (see ShootingConfiguration).
    public static final double GATE_TO_FEED_DELAY_SECONDS = ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS;
    public static final double FEED_WINDOW_SECONDS = ShootingConfiguration.FEED_WINDOW_SECONDS;
    public static final double RECOVERY_SECONDS = ShootingConfiguration.RECOVERY_SECONDS;
    public static final double READY_STABLE_SECONDS = ShootingConfiguration.READY_STABLE_SECONDS;
    private boolean continuous = true;
    private boolean previousToggle;
    private final StaggeredFeedTiming stage1Timing = new StaggeredFeedTiming();

    public static FiringCoordinator firing() {
        FiringCoordinator firing = new FiringCoordinator(GATE_TO_FEED_DELAY_SECONDS,
                ShootingConfiguration.FEED_TO_EXIT_SECONDS, FEED_WINDOW_SECONDS,
                RECOVERY_SECONDS, READY_STABLE_SECONDS);
        firing.setImmediateContinuousFeed(true);
        return firing;
    }

    /** Call before updating firing. A mode change starts a fresh gate/readiness sequence. */
    public void updateSelection(double now, boolean toggle, FiringCoordinator firing) {
        if (toggle && !previousToggle) continuous = !continuous;
        previousToggle = toggle;
        if (firing.configureFeedMode(now, continuous, FEED_WINDOW_SECONDS,
                RECOVERY_SECONDS, READY_STABLE_SECONDS, true)) stage1Timing.reset();
    }

    /** Call after updating firing, including idle/inhibited loops so the next shot has a fresh timer. */
    public boolean feedStage1(double now, FiringCoordinator firing) {
        // Both modes start stage 1 with stage 2; staggered waits for the gate delay first.
        return stage1Timing.feedStage1(now, firing);
    }

    /** True between held staggered shots while the gate is closed: stage 1 runs, stage 2 stays off. */
    public boolean closedGateStage1(FiringCoordinator firing) {
        return firing.closedGateStage1();
    }

    /** Stage 2 (and stage 1 once allowed) feed only while the gate is open for feeding. */
    public boolean feedTransfers(FiringCoordinator firing, boolean gateReady) {
        return firing.feeding() && (continuous || gateReady);
    }

    public boolean continuous() { return continuous; }
}
