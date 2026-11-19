package org.firstinspires.ftc.teamcode;

/** Hold-to-run bench controls with selectable continuous or staggered shooting. */
public final class IntakeShooterTestController {
    // Robot-tuned staggered timing shared with competition and autonomous (see ShootingConfiguration):
    // gate delay 0.10 s, both stages feed together for 0.20 s, no recovery pause.
    public static final double GATE_TO_FEED_DELAY_SECONDS = ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS;
    public static final double FEED_WINDOW_SECONDS = ShootingConfiguration.FEED_WINDOW_SECONDS;
    public static final double RECOVERY_SECONDS = ShootingConfiguration.RECOVERY_SECONDS;
    private final FiringCoordinator firing = new FiringCoordinator(GATE_TO_FEED_DELAY_SECONDS,
            ShootingConfiguration.FEED_TO_EXIT_SECONDS, FEED_WINDOW_SECONDS,
            RECOVERY_SECONDS, ShootingConfiguration.READY_STABLE_SECONDS);
    private boolean armed, spinFlywheel, loadIndex, reverseIntake;
    private boolean forwardIntake, conflictingBumpers, closedGateStage1;
    private final StaggeredFeedTiming stage1Timing = new StaggeredFeedTiming();
    private boolean feedStage1;

    public static final double BENCH_YAW_TOLERANCE_DEG = 4.0;
    /** Preserve the independently requested bench readiness tolerance. */
    public static boolean isYawSettled(double errorDeg, double velocityDegPerSecond) {
        return RobotMath.finite(errorDeg, velocityDegPerSecond)
                && Math.abs(errorDeg) <= BENCH_YAW_TOLERANCE_DEG
                && Math.abs(velocityDegPerSecond) <= TurretSubsystem.YAW_VERIFY_MAX_VELOCITY_DEG_PER_SECOND;
    }

    public void update(double now, boolean load, boolean reverse, boolean spin, boolean feed,
                       boolean stop, boolean homed, boolean ready) {
        if (stop) armed = false;
        else if (!load && !reverse && !spin && !feed) armed = true;

        spinFlywheel = armed && !stop && spin;
        boolean inhibit = !armed || stop || reverse;
        String rejection = !homed ? "Waiting for turret home" : "Waiting for flywheel";
        if (firing.continuous()) {
            // Operator-requested continuous mode runs immediately until either trigger is released.
            firing.updateUnrestrictedFeed(now, armed && spinFlywheel && feed, inhibit);
        } else {
            // Requalify readiness between staggered feeds; retain the trigger/home interlocks.
            firing.update(now, armed && feed, false, inhibit, !spinFlywheel,
                    homed, ready, ready, rejection);
        }
        // In staggered mode stage 2 launches the top ball before stage 1 advances the queue.
        feedStage1 = stage1Timing.feedStage1(now, firing);
        loadIndex = armed && !stop && !reverse && !feed;
        // Between held staggered shots the gate is closed: keep stage 1 advancing the queue (stage 2 stays off).
        closedGateStage1 = armed && !stop && !reverse && feed && firing.closedGateStage1();
        forwardIntake = armed && !stop && load && !reverse;
        conflictingBumpers = load && reverse;
        reverseIntake = armed && !stop && reverse && !load;
    }

    public boolean armed() { return armed; }
    /** Settings changes stop outputs and require releasing the motor controls before re-arming. */
    public boolean configureShooting(double now, boolean continuous, double feedSeconds,
                                     double pauseSeconds, double readySeconds) {
        boolean changed = firing.configureAutomaticFeed(now, continuous, feedSeconds, pauseSeconds, readySeconds);
        if (changed) {
            armed = spinFlywheel = loadIndex = reverseIntake = false;
            forwardIntake = conflictingBumpers = closedGateStage1 = false;
            feedStage1 = false;
            stage1Timing.reset();
        }
        return changed;
    }
    public boolean spinFlywheel() { return spinFlywheel; }
    public boolean loadIndex() { return loadIndex; }
    public boolean feedTransfer(boolean gateReady) {
        return firing.feeding() && (firing.continuous() || gateReady);
    }
    public boolean feedStage1() { return feedStage1; }
    /** Stage 1 only runs (past full) while the gate is closed between held staggered shots. */
    public boolean closedGateStage1() { return closedGateStage1; }
    public double intakePower(boolean full) {
        if (conflictingBumpers) return 0.0;
        if (forwardIntake) return 1.0;
        if (reverseIntake) return -1.0;
        return loadIndex ? (full ? IntakeQueueController.FULL_QUEUE_INTAKE_POWER : 1.0) : 0.0;
    }
    public FiringCoordinator firing() { return firing; }
}
