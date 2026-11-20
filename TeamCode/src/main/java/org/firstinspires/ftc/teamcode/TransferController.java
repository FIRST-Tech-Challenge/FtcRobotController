package org.firstinspires.ftc.teamcode;

/**
 * Deterministic full confirmation and stage-1 feed/loading priority. Stage 1 is the only transfer stage:
 * both transfer motors are mechanically linked to it and receive this one command (since 2026-11-18; the
 * back rollers transfer_motor_2 used to drive as a separate stage 2 were removed).
 */
public final class TransferController {
    private final double power, confirmationSeconds;
    public TransferController() { this(TransferSubsystem.TRANSFER_POWER, TransferSubsystem.FULL_CONFIRM_TIME_SECONDS); }
    public TransferController(double power, double confirmationSeconds) {
        if (!RobotMath.finite(power, confirmationSeconds) || power <= 0 || power > 1 || confirmationSeconds <= 0)
            throw new IllegalArgumentException("Invalid transfer configuration");
        this.power = power; this.confirmationSeconds = confirmationSeconds;
    }
    private double highSince = Double.NaN;
    private double runOnUntil = Double.NEGATIVE_INFINITY;
    private boolean full;
    private double stage1;
    /** Continuous HIGH for the confirmation time means full; any LOW clears full immediately. */
    public void observe(double now, boolean sensorHigh) {
        boolean wasFull = full();
        if (sensorHigh) {
            if (Double.isNaN(highSince)) highSince = now;
            full = now - highSince >= confirmationSeconds;
        } else { highSince = Double.NaN; full = false; runOnUntil = Double.NEGATIVE_INFINITY; }
        // Start once when a loading queue becomes full so stage 1 seats the fourth ball.
        if (!wasFull && full() && stage1 > 0)
            runOnUntil = now + TransferSubsystem.STAGE_1_FULL_RUN_ON_SECONDS;
    }
    public void update(double now, boolean sensorHigh, boolean feed, boolean pause) {
        update(now, sensorHigh, feed, pause, true);
    }
    /**
     * {@code feedStage1} false holds stage 1 off during a feed; the shared shot policy
     * ({@link StaggeredFeedTiming}) passes true for every feed. Loading behavior is unchanged.
     */
    public void update(double now, boolean sensorHigh, boolean feed, boolean pause, boolean feedStage1) {
        update(now, sensorHigh, feed, pause, feedStage1, false);
    }
    /**
     * Loading runs stage 1 on briefly after full is confirmed, then stops it until the sensor reads LOW,
     * which resumes loading automatically. Forward intake override bypasses full for stage 1;
     * feed and loading pauses retain priority and cancel the run-on.
     */
    public void update(double now, boolean sensorHigh, boolean feed, boolean pause, boolean feedStage1,
                       boolean forceStage1Loading) {
        observe(now, sensorHigh);
        if (feed || pause) runOnUntil = Double.NEGATIVE_INFINITY;
        // Caller owns shot policy, including competition gate-closed recovery feeding.
        stage1 = feed ? (feedStage1 ? power : 0)
                : (!pause && (forceStage1Loading || !full() || now < runOnUntil)) ? power : 0;
    }
    public void stop() { stage1 = 0; highSince = Double.NaN; full = false; runOnUntil = Double.NEGATIVE_INFINITY; }
    /** Pause outputs without declaring the queue empty or restarting HIGH confirmation. */
    public void pause() { stage1 = 0; runOnUntil = Double.NEGATIVE_INFINITY; }
    public boolean full() { return full; }
    /** The one transfer command, applied to both linked transfer motors. */
    public double stage1() { return stage1; }
    public double confirmingSeconds(double now) { return Double.isNaN(highSince) ? 0 : Math.max(0, now - highSince); }
}
