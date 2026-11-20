package org.firstinspires.ftc.teamcode;

/**
 * Sequencing of one path-planner volley or shot on top of TeleOp's firing, hardware-free: AIMING (TeleOp RT + LT
 * held: the shared {@link FiringCoordinator} feeds once the turret, hood and flywheel are ready and stable for
 * {@link CompetitionFeedController#READY_STABLE_SECONDS}), then FEEDING for the feed time, then DONE. If the robot is
 * not ready within the ready timeout it either fires anyway (TeleOp's manual feed, readiness bypassed; only with a
 * homed turret, so it never fires in an unknown direction) or skips the shot.
 *
 * <p>The robot cannot count launched balls (the transfer sensor only confirms a full queue), so a feed ends on time,
 * never on "queue empty". {@link PlannerRobot} updates this once per loop before the shot pipeline.
 */
public final class PlannerVolley {
    public enum Phase { IDLE, AIMING, FEEDING, DONE, SKIPPED }
    /** What to do when the robot is not ready within the ready timeout. */
    public enum TimeoutAction { FIRE, SKIP }

    private Phase phase = Phase.IDLE;
    private TimeoutAction onTimeout = TimeoutAction.FIRE;
    private double started, feedStarted, feedSeconds, readyTimeoutSeconds;
    private boolean forced, waitForReady;
    private String note = "";

    /**
     * @param feedSeconds          continuous feed time once feeding starts (s, > 0)
     * @param readyTimeoutSeconds  longest wait for readiness (s, >= 0)
     * @param waitForReady         false: feed at once with readiness bypassed (TeleOp GP1 A)
     */
    public void start(double now, double feedSeconds, double readyTimeoutSeconds, TimeoutAction onTimeout,
                      boolean waitForReady) {
        if (!(feedSeconds > 0) || !(readyTimeoutSeconds >= 0) || !Double.isFinite(feedSeconds)
                || !Double.isFinite(readyTimeoutSeconds) || onTimeout == null)
            throw new IllegalArgumentException("Invalid volley timing");
        this.feedSeconds = feedSeconds;
        this.readyTimeoutSeconds = readyTimeoutSeconds;
        this.onTimeout = onTimeout;
        this.waitForReady = waitForReady;
        started = now;
        forced = !waitForReady;
        if (waitForReady) { phase = Phase.AIMING; note = "AIMING: waiting for turret / hood / RPM"; }
        else { phase = Phase.FEEDING; feedStarted = now; note = "MANUAL FEED (readiness bypassed)"; }
    }

    /**
     * Once per loop, before this loop's shot pipeline.
     *
     * @param feeding      the firing coordinator fed on its last update (gate open, transfer feeding)
     * @param turretHomed  the turret has a valid home (its yaw is known)
     */
    public void update(double now, boolean feeding, boolean turretHomed) {
        if (phase == Phase.AIMING) {
            if (feeding) {
                phase = Phase.FEEDING; feedStarted = now; note = "READY: feeding";
            } else if (now - started >= readyTimeoutSeconds) {
                if (onTimeout == TimeoutAction.FIRE && turretHomed) {
                    forced = true; phase = Phase.FEEDING; feedStarted = now;
                    note = "fed before READY (timeout)";
                } else {
                    phase = Phase.SKIPPED;
                    note = onTimeout == TimeoutAction.FIRE ? "SKIPPED: turret not homed" : "SKIPPED: not ready in time";
                }
            }
        }
        if (phase == Phase.FEEDING && now - feedStarted >= feedSeconds) {
            phase = Phase.DONE;
            if (note.startsWith("READY") || note.startsWith("MANUAL")) note = "DONE";
        }
    }

    /** End now (stop all, a new volley, OpMode end). */
    public void cancel(String reason) {
        if (active()) { phase = Phase.SKIPPED; note = "CANCELLED: " + reason; }
    }

    /** TeleOp RT + LT: readiness-gated automatic feed. */
    public boolean automaticFeed() { return phase == Phase.AIMING || (phase == Phase.FEEDING && !forced); }
    /** TeleOp GP1 A: continuous feed with readiness bypassed (timeout FIRE, or no ready wait). */
    public boolean manualFeed() { return phase == Phase.FEEDING && forced; }
    public boolean active() { return phase == Phase.AIMING || phase == Phase.FEEDING; }
    public boolean finished() { return phase == Phase.DONE || phase == Phase.SKIPPED; }
    public boolean waitedForReady() { return waitForReady; }
    public Phase phase() { return phase; }
    public String note() { return note; }
}
