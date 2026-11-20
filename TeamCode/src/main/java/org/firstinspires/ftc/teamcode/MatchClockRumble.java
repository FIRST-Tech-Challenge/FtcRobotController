package org.firstinspires.ftc.teamcode;

/**
 * Hardware-free match-clock rumble cues for competition TeleOp. It decides <i>when</i> to rumble; the
 * OpMode makes the gamepad calls. Every cue fires once per event, never every loop.
 *
 * <ul>
 *     <li><b>Endgame warning</b>: both gamepads, {@link #ENDGAME_WARNING_BLIPS} short blips at
 *         {@link #ENDGAME_WARNING_SECONDS_LEFT} s left in the {@link #TELEOP_SECONDS} s TeleOp period.</li>
 *     <li><b>Final warning</b>: both gamepads, one {@link #FINAL_WARNING_RUMBLE_MS} ms rumble at
 *         {@link #FINAL_WARNING_SECONDS_LEFT} s left.</li>
 * </ul>
 *
 * The match clock counts from TeleOp START, like the Driver Station's 2:00 TeleOp timer. Shot-ready and
 * queue-full feedback is {@link DriverRumble}'s job (GP1 aiming rumble and full-queue pulses).
 */
public final class MatchClockRumble {
    // STARTING VALUES: adjust the warning times to the drive team's endgame plan.
    public static final double TELEOP_SECONDS = 120.0;
    public static final double ENDGAME_WARNING_SECONDS_LEFT = 20.0;
    public static final int ENDGAME_WARNING_BLIPS = 2;
    public static final double FINAL_WARNING_SECONDS_LEFT = 5.0;
    public static final int FINAL_WARNING_RUMBLE_MS = 1000;

    private double startedAt = Double.NaN;
    private boolean endgameDone, finalDone;
    private boolean endgameWarning, finalWarning;

    /** Call once at TeleOp START (System.nanoTime clock, seconds). */
    public void start(double now) {
        startedAt = now;
        endgameDone = finalDone = false;
    }

    /** Call once per loop. Read this loop's cues with the getters afterward. */
    public void update(double now) {
        endgameWarning = finalWarning = false;
        if (Double.isNaN(startedAt)) return;
        double left = secondsLeft(now);
        if (!finalDone && left <= FINAL_WARNING_SECONDS_LEFT) {
            // A late START past both marks gets only the most urgent cue.
            finalWarning = true; finalDone = endgameDone = true;
        } else if (!endgameDone && left <= ENDGAME_WARNING_SECONDS_LEFT) {
            endgameWarning = true; endgameDone = true;
        }
    }

    /** Seconds left in the TeleOp period (0 when over; the full period before START). */
    public double secondsLeft(double now) {
        if (Double.isNaN(startedAt)) return TELEOP_SECONDS;
        return Math.max(0, TELEOP_SECONDS - (now - startedAt));
    }

    public boolean endgameWarning() { return endgameWarning; }
    public boolean finalWarning() { return finalWarning; }
}
