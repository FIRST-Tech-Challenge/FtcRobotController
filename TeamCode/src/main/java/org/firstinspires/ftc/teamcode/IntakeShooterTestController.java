package org.firstinspires.ftc.teamcode;

/**
 * Bench intake/index/shooter controls running the competition TeleOp's shooting policy (ported 2026-11-19).
 *
 * <p>Shared with {@link MecanumTeleOp} through {@link CompetitionFeedController}: CONTINUOUS feed only (no
 * staggered mode), automatic RT + LT shots qualified for {@link #READY_STABLE_SECONDS} that keep feeding while the
 * looser feed-continuation check holds, an explicit manual feed that bypasses readiness, an operator inhibit that
 * wins over every feed, and automatic collection / stage-1 loading paused while shooting with the bumper overrides
 * retained.
 *
 * <p>Bench-only differences: every feed (automatic or manual) commands the gate open first and starts transfer
 * stage 1 {@link #GATE_OPEN_TO_TRANSFER_SECONDS} later (competition starts both together); motor controls arm only
 * after Back, B, bumpers and triggers are released; Back stops and disarms; the flywheel runs at the selected RPM
 * only while RT is held and is otherwise off (the competition TeleOp's {@link MecanumTeleOp#FLYWHEEL_IDLE_RPM}
 * idle is not used here). Without odometry or a camera the bench substitutes its own readiness (home, measured
 * RPM, bench yaw tolerance, hood) for the competition auto-aim solution.
 */
public final class IntakeShooterTestController {
    public static final double READY_STABLE_SECONDS = CompetitionFeedController.READY_STABLE_SECONDS;
    /** Bench-only (2026-11-19): transfer stage 1 starts this long after the gate-open command. */
    public static final double GATE_OPEN_TO_TRANSFER_SECONDS = 0.10;
    /** Preserve the independently requested bench readiness tolerance. */
    public static final double BENCH_YAW_TOLERANCE_DEG = 4.0;
    public static final double RPM_COARSE_STEP = 100.0;
    public static final double RPM_FINE_STEP = 10.0;

    // Competition continuous timing, except the gate-open-to-transfer delay is honored (competition's coordinator
    // feeds immediately): GATE_OPENING holds the gate open with the transfer off, then FEEDING runs stage 1.
    private final FiringCoordinator firing = new FiringCoordinator(GATE_OPEN_TO_TRANSFER_SECONDS,
            ShootingConfiguration.FEED_TO_EXIT_SECONDS, CompetitionFeedController.FEED_WINDOW_SECONDS,
            CompetitionFeedController.RECOVERY_SECONDS, READY_STABLE_SECONDS);
    private final CompetitionFeedController feedControl = new CompetitionFeedController();
    private final IntakeQueueController intakeControl = new IntakeQueueController();
    private boolean armed, active, spinFlywheel, inhibited, forward, reverse;
    private boolean autoShotReady, shootingBusy, feedStage1, closedGateStage1, manualRequest;

    /** Initial (tight) bench readiness: bench yaw error and turret rate limits. */
    public static boolean isYawSettled(double errorDeg, double velocityDegPerSecond) {
        return RobotMath.finite(errorDeg, velocityDegPerSecond)
                && Math.abs(errorDeg) <= BENCH_YAW_TOLERANCE_DEG
                && Math.abs(velocityDegPerSecond) <= TurretSubsystem.YAW_VERIFY_MAX_VELOCITY_DEG_PER_SECOND;
    }

    /**
     * Feed continuation, like the competition loose check: the yaw error and hood still hold and the flywheel is
     * within {@link TurretSubsystem#SHOOTER_LOOSE_READY_TOLERANCE_RPM} of its target, with no turret-rate limit.
     */
    public static boolean isLooseReady(double yawErrorDeg, boolean hoodSettled, double selectedRpm,
                                       double measuredRpm) {
        return RobotMath.finite(yawErrorDeg, selectedRpm, measuredRpm)
                && Math.abs(yawErrorDeg) <= BENCH_YAW_TOLERANCE_DEG && hoodSettled && selectedRpm > 0
                && Math.abs(measuredRpm - selectedRpm) <= TurretSubsystem.SHOOTER_LOOSE_READY_TOLERANCE_RPM;
    }

    /** One press-edge RPM change: coarse +/-100 and fine +/-10, clamped to 0..max flywheel RPM. */
    public static double stepRpm(double rpm, boolean coarseUp, boolean coarseDown, boolean fineUp, boolean fineDown) {
        double delta = ((coarseUp ? 1 : 0) - (coarseDown ? 1 : 0)) * RPM_COARSE_STEP
                + ((fineUp ? 1 : 0) - (fineDown ? 1 : 0)) * RPM_FINE_STEP;
        double next = (Double.isFinite(rpm) ? rpm : 0) + delta;
        return Math.max(0, Math.min(TurretSubsystem.SHOOTER_MAX_RPM, next));
    }

    /**
     * @param load        RB (either gamepad): force intake and stage-1 loading forward
     * @param reverseHeld LB (either gamepad): reverse intake only and pause stage-1 loading
     * @param spin        GP1 RT: run the flywheel at the selected RPM and prepare the automatic shot
     * @param feed        GP1 LT: with RT, fire automatically once ready
     * @param manualFeed  GP1 B or GP2 RT: feed immediately, readiness bypassed
     * @param inhibit     GP2 LT: force the gate closed and stop the transfer; overrides every feed
     * @param stop        GP1 Back, rehome or a controller switch: stop outputs and disarm until controls are released
     */
    public void update(double now, boolean load, boolean reverseHeld, boolean spin, boolean feed,
                       boolean manualFeed, boolean inhibit, boolean stop, boolean homed,
                       boolean tightReady, boolean looseReady) {
        if (stop) armed = false;
        else if (!load && !reverseHeld && !spin && !feed && !manualFeed) armed = true;
        active = armed && !stop;
        spinFlywheel = active && spin;
        inhibited = inhibit;
        forward = active && load;
        reverse = active && reverseHeld;
        boolean autoRequest = spinFlywheel && feed;
        manualRequest = active && manualFeed;

        feedControl.updateSelection(now, firing);
        // Same firing call as the competition ShotCoordinator; a stopped or unarmed bench inhibits like GP2 LT.
        firing.update(now, autoRequest, manualRequest, inhibit || !active, false, homed, tightReady, looseReady,
                homed ? "Waiting for flywheel" : "Waiting for turret home");
        autoShotReady = spinFlywheel && homed && tightReady && !inhibit;
        feedStage1 = feedControl.feedStage1(now, firing);
        closedGateStage1 = feedControl.closedGateStage1(firing) && !inhibit && !reverse;
        shootingBusy = autoRequest || manualRequest || firing.gateOpen()
                || firing.state() == FiringCoordinator.State.RECOVERY;
    }

    public boolean armed() { return armed; }
    /** Armed with RT held: run the selected RPM. Otherwise the bench flywheel is off (no idle). */
    public boolean spinFlywheel() { return spinFlywheel; }
    /** RT held and the shot would qualify now (drives the GP1 rumble and shooter light like competition). */
    public boolean autoShotReady() { return autoShotReady; }
    public boolean manualFeedRequested() { return manualRequest; }
    public boolean shootingBusy() { return shootingBusy; }
    /**
     * The transfer feeds once firing reaches FEEDING, {@link #GATE_OPEN_TO_TRANSFER_SECONDS} after the gate-open
     * command; the coordinator owns that delay, so the gate's own readiness flag is not needed.
     */
    public boolean feedTransfer(boolean gateReady) { return feedControl.feedTransfers(firing, gateReady); }
    public boolean feedStage1() { return feedStage1; }
    /** Never true in continuous mode; kept so the transfer logic matches the competition loop. */
    public boolean closedGateStage1() { return closedGateStage1; }
    /** Competition loading pause: inhibit, any shot activity or reverse, plus a stopped/unarmed bench. */
    public boolean pauseLoading() {
        return !closedGateStage1 && (!active || inhibited || shootingBusy || reverse);
    }
    /** Competition RB override: stage 1 loads past full while forward is held without reverse. */
    public boolean forceStage1Loading() { return closedGateStage1 || (forward && !reverse); }
    public double intakePower(boolean full) {
        return active ? intakeControl.update(full, forward, reverse, inhibited || shootingBusy) : 0.0;
    }
    public FiringCoordinator firing() { return firing; }
}
