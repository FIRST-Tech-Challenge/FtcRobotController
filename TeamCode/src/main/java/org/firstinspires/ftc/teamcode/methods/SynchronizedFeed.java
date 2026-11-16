package org.firstinspires.ftc.teamcode.methods;

/**
 * Stored-batch feeding.
 *
 * CONTINUOUS (original behavior): both feed motors run nonstop once the flywheel is ready,
 * so balls 2-4 meet a flywheel that has not recovered from the previous ball.
 *
 * PACED: each ball contact is detected from the flywheel speed dip. Transfer and intake
 * stop immediately, the flywheel is allowed to recover, then the transfer ramps again and
 * advances the next ball. Every ball therefore starts from (nearly) the same wheel speed.
 *
 * Shots are detected and counted in both modes so they can be logged and compared.
 * The encoder dip is an estimate of ball contact, not proof that a ball scored.
 */
public final class SynchronizedFeed {
    public enum State { IDLE, WAIT_FLYWHEEL, RAMP_TRANSFER, FEED, RECOVER, FAULT }
    public enum Mode { CONTINUOUS, PACED }

    public static final double TRANSFER_TARGET = 1440, RAMP_SECONDS = .100;
    public static final double FLYWHEEL_TOLERANCE = 25, FLYWHEEL_DWELL = .200;

    // Shot detection. Logged dips were 60-150 ticks/s; steady-state noise is about +/-20.
    /** A drop this far below min(recent peak, target) counts as a ball contact. */
    public static final double SHOT_DROP = 60;
    /** After a contact, the wheel must rise this far above the dip minimum to re-arm. */
    public static final double SHOT_REARM_RISE = 40;

    // Paced-mode defaults; a test can change them per instance while idle.
    public static final double PACED_RECOVER_TOLERANCE = 25;
    public static final double PACED_RECOVER_DWELL = .050;
    /** If the wheel never settles, resume anyway rather than stalling the batch. */
    public static final double PACED_MAX_RECOVER_SECONDS = .800;

    private State state = State.IDLE;
    private double phaseStart, lastTime = Double.NaN, fStable = Double.NaN;
    private double transferTarget, feedSeconds;
    private boolean intakeEnabled;
    private String fault = "";
    private double rampSeconds = RAMP_SECONDS;

    private Mode mode = Mode.CONTINUOUS;
    private double transferSetpoint = TRANSFER_TARGET;
    private double pacedTolerance = PACED_RECOVER_TOLERANCE;
    private double pacedDwell = PACED_RECOVER_DWELL;
    private double pacedMaxRecover = PACED_MAX_RECOVER_SECONDS;
    private double pacedStable = Double.NaN;

    // Shot detector
    private boolean inDip;
    private double peakSinceArm = Double.NaN, dipMin = Double.NaN;
    private int shotCount, recoverTimeouts;
    private boolean shotThisUpdate;
    private double lastShotTime = Double.NaN, lastRecoverSeconds = Double.NaN;

    /** Per-instance experiment setting; never change a batch already in progress. */
    public void setRampSeconds(double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0) throw new IllegalArgumentException("Positive finite ramp required");
        requireIdle();
        rampSeconds = seconds;
    }

    public void setMode(Mode newMode) {
        if (newMode == null) throw new IllegalArgumentException("Mode required");
        requireIdle();
        mode = newMode;
    }

    /** Transfer speed while feeding, ticks/s. Lower values space balls further apart. */
    public void setTransferSetpoint(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond <= 0)
            throw new IllegalArgumentException("Positive finite transfer setpoint required");
        requireIdle();
        transferSetpoint = ticksPerSecond;
    }

    /** Paced recovery: |target - flywheel| <= tolerance held for dwell; resume anyway after maxSeconds. */
    public void setPacedRecovery(double tolerance, double dwellSeconds, double maxSeconds) {
        if (!(tolerance > 0) || !(dwellSeconds >= 0) || !(maxSeconds > 0)
                || !Double.isFinite(tolerance) || !Double.isFinite(dwellSeconds) || !Double.isFinite(maxSeconds))
            throw new IllegalArgumentException("Finite tolerance > 0, dwell >= 0, max > 0 required");
        requireIdle();
        pacedTolerance = tolerance;
        pacedDwell = dwellSeconds;
        pacedMaxRecover = maxSeconds;
    }

    private void requireIdle() {
        if (state != State.IDLE) throw new IllegalStateException("Change feed settings only while idle");
    }

    public State state() { return state; }
    public Mode mode() { return mode; }
    public double transferSetpoint() { return transferSetpoint; }
    public double transferTarget() { return transferTarget; }
    public boolean intakeEnabled() { return intakeEnabled; }
    public double feedSeconds() { return feedSeconds; }
    public String fault() { return fault; }
    /** Ball contacts detected since the current request began. */
    public int shotCount() { return shotCount; }
    /** True only on the update() that detected a new contact. */
    public boolean shotDetected() { return shotThisUpdate; }
    public double lastShotTime() { return lastShotTime; }
    /** PACED only: seconds the most recent RECOVER phase lasted. */
    public double lastRecoverSeconds() { return lastRecoverSeconds; }
    /** PACED only: recoveries that hit the max time and resumed without settling. */
    public int recoverTimeouts() { return recoverTimeouts; }

    public void cancel() {
        transferTarget = 0; intakeEnabled = false;
        fStable = Double.NaN; pacedStable = Double.NaN; lastTime = Double.NaN;
        shotThisUpdate = false;
        if (state != State.FAULT) state = State.IDLE;
    }
    public void abort(String reason) { fault = reason; state = State.FAULT; cancel(); }
    public void clearFault() { fault = ""; state = State.IDLE; cancel(); }

    public void update(double now, boolean request, boolean wheelEnabled,
                       double target, double flywheel, double transfer) {
        update(now, request, wheelEnabled, target, flywheel, transfer, true);
    }

    /** TeleOp can request feeding independently of flywheel enable/readiness. */
    public void update(double now, boolean request, boolean wheelEnabled,
                       double target, double flywheel, double transfer, boolean waitForFlywheel) {
        boolean wasFeeding = intakeEnabled;
        transferTarget = 0; intakeEnabled = false; shotThisUpdate = false;
        if (state == State.FAULT) return;
        if (!Double.isFinite(now)) now = Double.isFinite(lastTime) ? lastTime + .02 : 0;
        if (Double.isFinite(lastTime) && now <= lastTime) now = lastTime + .02;
        double dt = now - lastTime;
        if (wasFeeding && Double.isFinite(dt) && dt > 0) feedSeconds += dt;
        lastTime = now;
        if (!request || (waitForFlywheel && (!wheelEnabled || target == 0))) { cancel(); return; }
        if (state == State.IDLE) {
            lastTime = now;
            resetShotDetector();
            transition(waitForFlywheel ? State.WAIT_FLYWHEEL : State.RAMP_TRANSFER, now);
        }
        boolean fReady = Math.abs(target - flywheel) <= FLYWHEEL_TOLERANCE;
        fStable = since(fReady, fStable, now);
        if (state == State.RAMP_TRANSFER || state == State.FEED || state == State.RECOVER) {
            shotThisUpdate = detectShot(now, target, flywheel);
        }
        if (mode == Mode.PACED && shotThisUpdate
                && (state == State.RAMP_TRANSFER || state == State.FEED)) {
            // Stop pushing the next ball while the wheel recovers from this one.
            transition(State.RECOVER, now);
            pacedStable = Double.NaN;
        }
        double elapsed = now - phaseStart;
        switch (state) {
            case WAIT_FLYWHEEL:
                if (held(fStable, now, FLYWHEEL_DWELL)) {
                    transition(State.RAMP_TRANSFER, now);
                }
                break;
            case RAMP_TRANSFER:
                transferTarget = transferSetpoint * Math.min(1, elapsed / rampSeconds);
                intakeEnabled = transferTarget > 0;
                if (elapsed >= rampSeconds) transition(State.FEED, now);
                break;
            case FEED:
                // CONTINUOUS empties the whole tunnel through shot dips.
                // PACED leaves FEED above as soon as a contact is detected.
                transferTarget = transferSetpoint; intakeEnabled = true;
                break;
            case RECOVER:
                if (mode == Mode.PACED) {
                    boolean settled = Math.abs(target - flywheel) <= pacedTolerance;
                    pacedStable = since(settled, pacedStable, now);
                    boolean timedOut = elapsed >= pacedMaxRecover;
                    if (held(pacedStable, now, pacedDwell) || timedOut) {
                        if (timedOut && !held(pacedStable, now, pacedDwell)) recoverTimeouts++;
                        lastRecoverSeconds = elapsed;
                        transition(State.RAMP_TRANSFER, now);
                        // The ramp starts from zero on the next update.
                        transferTarget = 0; intakeEnabled = false;
                    }
                } else if (held(fStable, now, FLYWHEEL_DWELL)) {
                    transition(State.RAMP_TRANSFER, now);
                }
                break;
            default: break;
        }
        if (state == State.FAULT) { transferTarget = 0; intakeEnabled = false; }
    }

    private void resetShotDetector() {
        inDip = false; peakSinceArm = Double.NaN; dipMin = Double.NaN;
        shotCount = 0; recoverTimeouts = 0;
        lastShotTime = Double.NaN; lastRecoverSeconds = Double.NaN;
    }

    private boolean detectShot(double now, double target, double flywheel) {
        if (!Double.isFinite(flywheel) || !Double.isFinite(target) || target <= 0) return false;
        if (!inDip) {
            if (Double.isNaN(peakSinceArm) || flywheel > peakSinceArm) peakSinceArm = flywheel;
            // Reference never exceeds target, so a noisy high sample cannot fake a dip.
            double reference = Math.min(peakSinceArm, target);
            if (reference - flywheel >= SHOT_DROP) {
                inDip = true; dipMin = flywheel;
                shotCount++; lastShotTime = now;
                return true;
            }
        } else {
            dipMin = Math.min(dipMin, flywheel);
            if (flywheel - dipMin >= SHOT_REARM_RISE) {
                inDip = false; peakSinceArm = flywheel;
            }
        }
        return false;
    }

    private void transition(State next, double now) { state = next; phaseStart = now; }
    private static double since(boolean value, double since, double now) {
        return value ? Double.isNaN(since) ? now : since : Double.NaN;
    }
    private static boolean held(double since, double now, double duration) {
        return Double.isFinite(since) && now - since >= duration;
    }
}
