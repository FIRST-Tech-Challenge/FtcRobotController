package org.firstinspires.ftc.teamcode;

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
    }
}
