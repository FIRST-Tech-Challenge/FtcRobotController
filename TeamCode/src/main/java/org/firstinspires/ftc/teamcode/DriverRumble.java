package org.firstinspires.ftc.teamcode;

/**
 * Hardware-free controller-rumble cues for competition TeleOp. It decides <i>when</i> to rumble; the
 * OpMode makes the gamepad calls. Every cue fires once per event, never every loop.
 *
 * <ul>
 *     <li><b>Endgame warning</b>: both gamepads, {@link #ENDGAME_WARNING_BLIPS} short blips at
 *         {@link #ENDGAME_WARNING_SECONDS_LEFT} s left in the {@link #TELEOP_SECONDS} s TeleOp period.</li>
 *     <li><b>Final warning</b>: both gamepads, one {@link #FINAL_WARNING_RUMBLE_MS} ms rumble at
 *         {@link #FINAL_WARNING_SECONDS_LEFT} s left.</li>
 *     <li><b>Shot ready</b>: Gamepad 1, one short rumble when automatic shooting becomes READY
 *         (rising edge), at most once per {@link #SHOT_READY_MIN_INTERVAL_SECONDS}.</li>
 *     <li><b>Transfer full</b>: Gamepad 1, one {@link #TRANSFER_FULL_RUMBLE_MS} ms rumble when the transfer
 *         confirms a full queue (rising edge), at most once per {@link #TRANSFER_FULL_MIN_INTERVAL_SECONDS}.</li>
 * </ul>
 *
 * The match clock counts from TeleOp START, like the Driver Station's 2:00 TeleOp timer.
 */
public final class DriverRumble {
    // STARTING VALUES: adjust the warning times to the drive team's endgame plan.
    public static final double TELEOP_SECONDS = 120.0;
    public static final double ENDGAME_WARNING_SECONDS_LEFT = 20.0;
    public static final int ENDGAME_WARNING_BLIPS = 2;
    public static final double FINAL_WARNING_SECONDS_LEFT = 5.0;
    public static final int FINAL_WARNING_RUMBLE_MS = 1000;
    public static final int SHOT_READY_RUMBLE_MS = 150;
    // READY can flicker while aiming; one buzz per second at most.
    public static final double SHOT_READY_MIN_INTERVAL_SECONDS = 1.0;
    // Longer than the READY buzz so the driver can tell them apart: "stop collecting, go shoot".
    public static final int TRANSFER_FULL_RUMBLE_MS = 400;
    // Full is already confirmed after 0.2 s HIGH; this only guards a ball rattling at the sensor.
    public static final double TRANSFER_FULL_MIN_INTERVAL_SECONDS = 1.0;

    private double startedAt = Double.NaN;
    private boolean endgameDone, finalDone, previousShotReady;
    private double lastShotReadyRumble = Double.NEGATIVE_INFINITY;
    private boolean previousTransferFull;
    private double lastTransferFullRumble = Double.NEGATIVE_INFINITY;
    private boolean endgameWarning, finalWarning, shotReady, transferFull;

    /** Call once at TeleOp START (System.nanoTime clock, seconds). */
    public void start(double now) {
        startedAt = now;
        endgameDone = finalDone = previousShotReady = false;
        lastShotReadyRumble = Double.NEGATIVE_INFINITY;
        previousTransferFull = false;
        lastTransferFullRumble = Double.NEGATIVE_INFINITY;
    }

    /** Call once per loop. Read this loop's cues with the getters afterward. */
    public void update(double now, boolean automaticShotReady, boolean transferFullConfirmed) {
        endgameWarning = finalWarning = shotReady = transferFull = false;
        if (Double.isNaN(startedAt)) return;
        double left = secondsLeft(now);
        if (!finalDone && left <= FINAL_WARNING_SECONDS_LEFT) {
            // A late START past both marks gets only the most urgent cue.
            finalWarning = true; finalDone = endgameDone = true;
        } else if (!endgameDone && left <= ENDGAME_WARNING_SECONDS_LEFT) {
            endgameWarning = true; endgameDone = true;
        }
        if (automaticShotReady && !previousShotReady
                && now - lastShotReadyRumble >= SHOT_READY_MIN_INTERVAL_SECONDS) {
            shotReady = true; lastShotReadyRumble = now;
        }
        previousShotReady = automaticShotReady;
        if (transferFullConfirmed && !previousTransferFull
                && now - lastTransferFullRumble >= TRANSFER_FULL_MIN_INTERVAL_SECONDS) {
            transferFull = true; lastTransferFullRumble = now;
        }
        previousTransferFull = transferFullConfirmed;
    }

    /** Seconds left in the TeleOp period (0 when over; the full period before START). */
    public double secondsLeft(double now) {
        if (Double.isNaN(startedAt)) return TELEOP_SECONDS;
        return Math.max(0, TELEOP_SECONDS - (now - startedAt));
    }

    public boolean endgameWarning() { return endgameWarning; }
    public boolean finalWarning() { return finalWarning; }
    public boolean shotReady() { return shotReady; }
    public boolean transferFull() { return transferFull; }
}
