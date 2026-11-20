package org.firstinspires.ftc.teamcode;

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
}
