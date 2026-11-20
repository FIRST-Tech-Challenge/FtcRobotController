package org.firstinspires.ftc.teamcode;

<<<<<<< Updated upstream
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
=======
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * GP1 (driver) rumble for both competition TeleOps:
 * <ul>
 *     <li>Transfer queue just became full: two short strong pulses.</li>
 *     <li>RT (auto aim) held and the shot is not ready yet: steady light rumble, which stops the moment the shot is
 *         ready or RT is released. If readiness is then lost for {@link #NOT_READY_RESUME_SECONDS} with RT still
 *         held, the rumble resumes.</li>
 * </ul>
 * The gamepad is commanded only when the wanted output changes, never every loop. The full-queue pulses play out
 * before the aiming rumble resumes. Plain logic except {@link #update}, which queues SDK rumble commands
 * (non-blocking). Logitech F310 controllers have no rumble motors and ignore it.
 */
public final class DriverRumble {
    public enum Output { OFF, AIMING, QUEUE_FULL }

    /** Steady rumble power (0-1, both motors) while RT is held and the shot is not ready. */
    public static final double AIMING_RUMBLE_POWER = 0.4;
    public static final double QUEUE_FULL_PULSE_POWER = 1.0;
    public static final int QUEUE_FULL_PULSE_MS = 150, QUEUE_FULL_GAP_MS = 120;
    public static final double QUEUE_FULL_CUE_SECONDS = (2 * QUEUE_FULL_PULSE_MS + QUEUE_FULL_GAP_MS) / 1000.0;
    /** After the shot was ready, readiness must stay lost this long before the aiming rumble resumes (no chatter). */
    public static final double NOT_READY_RESUME_SECONDS = 0.25;

    private static final Gamepad.RumbleEffect QUEUE_FULL_EFFECT = new Gamepad.RumbleEffect.Builder()
            .addStep(QUEUE_FULL_PULSE_POWER, QUEUE_FULL_PULSE_POWER, QUEUE_FULL_PULSE_MS)
            .addStep(0, 0, QUEUE_FULL_GAP_MS)
            .addStep(QUEUE_FULL_PULSE_POWER, QUEUE_FULL_PULSE_POWER, QUEUE_FULL_PULSE_MS)
            .build();

    private Boolean full; // Null until the first reading: that one is a baseline, never a cue.
    private boolean aimingRumble, readyThisAim;
    private double notReadySince = Double.NaN;
    private double queueFullUntil = Double.NEGATIVE_INFINITY;
    private Output output = Output.OFF;
    private int queueFullCues;

    /** Call once per ACTIVE loop; commands GP1 only when the wanted rumble changes. */
    public void update(double now, boolean queueFull, boolean aimHeld, boolean shotReady, Gamepad driver) {
        Output change = next(now, queueFull, aimHeld, shotReady);
        if (change == null || driver == null) return;
        switch (change) {
            case QUEUE_FULL: driver.runRumbleEffect(QUEUE_FULL_EFFECT); break;
            case AIMING: driver.rumble(AIMING_RUMBLE_POWER, AIMING_RUMBLE_POWER, Gamepad.RUMBLE_DURATION_CONTINUOUS); break;
            default: driver.stopRumble(); break;
        }
    }

    /** The new output to command now, or null when it is unchanged. {@link #update} uses this. */
    public Output next(double now, boolean queueFull, boolean aimHeld, boolean shotReady) {
        boolean fullRose = full != null && !full && queueFull;
        full = queueFull;

        if (!aimHeld) {
            aimingRumble = readyThisAim = false;
            notReadySince = Double.NaN;
        } else if (shotReady) {
            aimingRumble = false;
            readyThisAim = true;
            notReadySince = Double.NaN;
        } else if (!readyThisAim) {
            aimingRumble = true; // Rumble from the moment RT is pressed until the shot is first ready.
        } else {
            if (Double.isNaN(notReadySince)) notReadySince = now;
            if (now - notReadySince >= NOT_READY_RESUME_SECONDS) aimingRumble = true;
        }

        Output wanted;
        if (fullRose) {
            queueFullUntil = now + QUEUE_FULL_CUE_SECONDS;
            queueFullCues++;
            wanted = Output.QUEUE_FULL;
        } else if (now < queueFullUntil) {
            return null; // Let the full-queue pulses finish.
        } else {
            wanted = aimingRumble ? Output.AIMING : Output.OFF;
        }
        if (wanted == output && !fullRose) return null;
        output = wanted;
        return wanted;
    }

    public Output output() { return output; }

    /** Null-safe; for the OpMode's shutdown path. */
    public static void stop(Gamepad... pads) {
        for (Gamepad pad : pads) if (pad != null) pad.stopRumble();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Rumble GP1", "%s | queue-full cues %d", output, queueFullCues);
    }
>>>>>>> Stashed changes
}
