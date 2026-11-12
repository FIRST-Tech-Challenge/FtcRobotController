package org.firstinspires.ftc.teamcode;

/** Hold-to-run bench controls with selectable continuous or staggered shooting. */
public final class IntakeShooterTestController {
    // Bench-only tuning: start transfer 0.10 s earlier and end each feed 0.10 s sooner.
    public static final double GATE_TO_FEED_DELAY_SECONDS = 0.10;
    public static final double FEED_WINDOW_SECONDS = 0.40;
    private final FiringCoordinator firing = new FiringCoordinator(GATE_TO_FEED_DELAY_SECONDS,
            ShootingConfiguration.FEED_TO_EXIT_SECONDS, FEED_WINDOW_SECONDS,
            ShootingConfiguration.RECOVERY_SECONDS, ShootingConfiguration.READY_STABLE_SECONDS);
    private boolean armed, spinFlywheel, loadIndex, reverseIntake;
    private boolean forwardIntake, conflictingBumpers;
    private double gateOpenedAt = Double.NaN;
    private boolean feedStage1;

    // Compatibility aliases for the bench tuning display and existing callers.
    public static final double INTAKE_JAM_MIN_VELOCITY_TICKS_PER_SECOND = IntakeJamRecovery.MIN_VELOCITY_TICKS_PER_SECOND;
    public static final double INTAKE_JAM_DETECT_SECONDS = IntakeJamRecovery.DETECT_SECONDS;
    public static final double INTAKE_JAM_CLEAR_SECONDS = IntakeJamRecovery.CLEAR_SECONDS;
    public static final double INTAKE_JAM_DIRECTION_SECONDS = IntakeJamRecovery.DIRECTION_SECONDS;
    private final IntakeJamRecovery intakeRecovery = new IntakeJamRecovery();

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
        if (!firing.gateOpen()) gateOpenedAt = Double.NaN;
        else if (Double.isNaN(gateOpenedAt)) gateOpenedAt = now;
        // In staggered mode stage 2 launches the top ball before stage 1 advances the queue.
        feedStage1 = firing.feeding() && (firing.continuous()
                || now - gateOpenedAt >= (firing.gateTravelSeconds + firing.feedWindowSeconds) / 2.0);
        loadIndex = armed && !stop && !reverse && !feed;
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
            forwardIntake = conflictingBumpers = false;
            feedStage1 = false;
            gateOpenedAt = Double.NaN;
        }
        return changed;
    }
    public boolean spinFlywheel() { return spinFlywheel; }
    public boolean loadIndex() { return loadIndex; }
    public boolean feedTransfer(boolean gateReady) {
        return firing.feeding() && (firing.continuous() || gateReady);
    }
    public boolean feedStage1() { return feedStage1; }
    public double intakePower(boolean full) {
        if (conflictingBumpers) return 0.0;
        if (forwardIntake) return 1.0;
        if (reverseIntake) return -1.0;
        return loadIndex ? (full ? IntakeQueueController.FULL_QUEUE_INTAKE_POWER : 1.0) : 0.0;
    }

    /** Call once per loop after updating controls; pass this processed power directly to the motor. */
    public double updateIntakePower(double now, boolean full, double velocityTicksPerSecond) {
        return intakeRecovery.update(now, intakePower(full), velocityTicksPerSecond);
    }

    public boolean isIntakeClearing() { return intakeRecovery.isClearing(); }
    public FiringCoordinator firing() { return firing; }
}
