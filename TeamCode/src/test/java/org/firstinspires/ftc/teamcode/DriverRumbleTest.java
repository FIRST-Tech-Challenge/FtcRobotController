package org.firstinspires.ftc.teamcode;

<<<<<<< Updated upstream
import org.junit.Test;
import static org.junit.Assert.*;

public class DriverRumbleTest {
    private static final double ENDGAME_AT = DriverRumble.TELEOP_SECONDS - DriverRumble.ENDGAME_WARNING_SECONDS_LEFT;
    private static final double FINAL_AT = DriverRumble.TELEOP_SECONDS - DriverRumble.FINAL_WARNING_SECONDS_LEFT;

    @Test public void endgameAndFinalWarningsFireExactlyOnceEach() {
        DriverRumble cues = new DriverRumble();
        cues.start(100);
        int endgame = 0, last = 0;
        for (double t = 100; t <= 100 + DriverRumble.TELEOP_SECONDS + 5; t += .125) {
            cues.update(t, false, false);
            if (cues.endgameWarning()) {
                endgame++;
                assertEquals(100 + ENDGAME_AT, t, 1e-9);
            }
            if (cues.finalWarning()) {
                last++;
                assertEquals(100 + FINAL_AT, t, 1e-9);
            }
        }
        assertEquals(1, endgame);
        assertEquals(1, last);
    }

    @Test public void cuesLastOnlyOneLoop() {
        DriverRumble cues = new DriverRumble();
        cues.start(0);
        cues.update(ENDGAME_AT, false, false);
        assertTrue(cues.endgameWarning());
        cues.update(ENDGAME_AT + .02, false, false);
        assertFalse(cues.endgameWarning());
    }

    @Test public void lateStartPastBothMarksGivesOnlyTheFinalWarning() {
        DriverRumble cues = new DriverRumble();
        cues.start(0);
        cues.update(FINAL_AT + 1, false, false);
        assertTrue(cues.finalWarning());
        assertFalse(cues.endgameWarning());
        cues.update(FINAL_AT + 2, false, false);
        assertFalse(cues.endgameWarning());
        assertFalse(cues.finalWarning());
    }

    @Test public void shotReadyBuzzesOnTheRisingEdgeWithAMinimumInterval() {
        DriverRumble cues = new DriverRumble();
        cues.start(0);
        cues.update(1, false, false);
        assertFalse(cues.shotReady());
        cues.update(2, true, false);
        assertTrue(cues.shotReady());
        cues.update(2.125, true, false);
        assertFalse("held READY does not keep buzzing", cues.shotReady());
        cues.update(2.25, false, false);
        cues.update(2.5, true, false); // Flicker inside the interval: silent.
        assertFalse(cues.shotReady());
        cues.update(2.625, false, false);
        cues.update(2 + DriverRumble.SHOT_READY_MIN_INTERVAL_SECONDS, true, false);
        assertTrue(cues.shotReady());
    }

    @Test public void transferFullRumblesOnceWhenTheQueueFillsAndAgainAfterARefill() {
        DriverRumble cues = new DriverRumble();
        cues.start(0);
        cues.update(1, false, false);
        assertFalse(cues.transferFull());
        cues.update(2, false, true);
        assertTrue(cues.transferFull());
        assertFalse("independent of the READY cue", cues.shotReady());
        cues.update(2.125, false, true);
        assertFalse("staying full does not keep rumbling", cues.transferFull());
        cues.update(2.25, false, false);
        cues.update(2.5, false, true); // Ball rattling at the sensor inside the interval: silent.
        assertFalse(cues.transferFull());
        cues.update(4, false, false); // Shot the queue empty...
        cues.update(6, false, true);  // ...and refilled it.
        assertTrue(cues.transferFull());
    }

    @Test public void nothingBeforeStartAndClockIsClamped() {
        DriverRumble cues = new DriverRumble();
        cues.update(500, true, false);
        assertFalse(cues.shotReady());
        assertFalse(cues.finalWarning());
        assertEquals(DriverRumble.TELEOP_SECONDS, cues.secondsLeft(500), 0);
        cues.start(0);
        assertEquals(DriverRumble.TELEOP_SECONDS - 30, cues.secondsLeft(30), 1e-9);
        assertEquals(0, cues.secondsLeft(DriverRumble.TELEOP_SECONDS + 10), 0);
=======
import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.DriverRumble.Output;
import org.junit.Test;

public class DriverRumbleTest {
    @Test public void queueFullPulsesOncePerFillAndTheFirstReadingIsABaseline() {
        DriverRumble r = new DriverRumble();
        assertNull(r.next(0, true, false, false)); // Already full at START: baseline, no cue.
        assertNull(r.next(1, false, false, false));
        assertEquals(Output.QUEUE_FULL, r.next(2, true, false, false));
        assertEquals(Output.OFF, r.next(2 + DriverRumble.QUEUE_FULL_CUE_SECONDS, true, false, false));
        assertNull(r.next(3, true, false, false)); // Held full: no repeat.
        r.next(4, false, false, false);
        assertEquals(Output.QUEUE_FULL, r.next(5, true, false, false));
    }

    @Test public void holdingRtRumblesUntilTheShotIsReady() {
        DriverRumble r = new DriverRumble();
        r.next(0, false, false, false);
        assertEquals(Output.AIMING, r.next(.1, false, true, false));
        assertNull(r.next(.2, false, true, false)); // Commanded once, not every loop.
        assertEquals(Output.OFF, r.next(.3, false, true, true));
        assertNull(r.next(.4, false, true, true));
        assertNull(r.next(1, false, false, false)); // RT released: already off.
        assertEquals(Output.AIMING, r.next(1.1, false, true, false));
        // RT released before the shot was ready: the rumble stops.
        assertEquals(Output.OFF, r.next(1.2, false, false, false));
    }

    @Test public void lostReadinessResumesTheRumbleOnlyAfterTheResumeDelay() {
        DriverRumble r = new DriverRumble();
        r.next(0, false, true, false);
        assertEquals(Output.OFF, r.next(.1, false, true, true));
        assertNull(r.next(.2, false, true, false)); // Flicker: still off.
        assertNull(r.next(.3, false, true, true));
        assertNull(r.next(.4, false, true, false));
        assertEquals(Output.AIMING, r.next(.4 + DriverRumble.NOT_READY_RESUME_SECONDS + .01, false, true, false));
        // A new RT press rumbles immediately again.
        assertEquals(Output.OFF, r.next(1, false, false, false));
        assertEquals(Output.AIMING, r.next(1.1, false, true, false));
    }

    @Test public void queueFullPulsesPlayOutThenTheAimingRumbleResumes() {
        DriverRumble r = new DriverRumble();
        r.next(0, false, false, false);
        assertEquals(Output.AIMING, r.next(.1, false, true, false));
        assertEquals(Output.QUEUE_FULL, r.next(.2, true, true, false));
        assertNull(r.next(.3, true, true, false)); // Pulses still playing.
        assertEquals(Output.AIMING, r.next(.2 + DriverRumble.QUEUE_FULL_CUE_SECONDS + .01, true, true, false));
    }

    @Test public void updateCommandsTheSdkGamepad() {
        DriverRumble r = new DriverRumble();
        Gamepad gp1 = new Gamepad();
        r.update(0, false, true, false, gp1);
        assertTrue(gp1.isRumbling()); // Continuous until stopped.
        Gamepad.RumbleEffect aiming = gp1.rumbleQueue.peek();
        assertEquals(1, aiming.steps.size());
        assertEquals(Gamepad.RUMBLE_DURATION_CONTINUOUS, aiming.steps.get(0).duration);
        assertEquals(102, aiming.steps.get(0).large); // 0.4 power.
        r.update(.1, false, true, true, gp1);
        assertFalse(gp1.isRumbling());
        r.update(.2, true, true, true, gp1);
        Gamepad.RumbleEffect full = gp1.rumbleQueue.peek();
        assertEquals(3, full.steps.size()); // Pulse, gap, pulse.
        assertEquals(255, full.steps.get(0).large);
        DriverRumble.stop(gp1, null);
        assertFalse(gp1.isRumbling());
>>>>>>> Stashed changes
    }
}
