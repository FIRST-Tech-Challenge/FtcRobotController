package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class MatchClockRumbleTest {
    private static final double ENDGAME_AT = MatchClockRumble.TELEOP_SECONDS - MatchClockRumble.ENDGAME_WARNING_SECONDS_LEFT;
    private static final double FINAL_AT = MatchClockRumble.TELEOP_SECONDS - MatchClockRumble.FINAL_WARNING_SECONDS_LEFT;

    @Test public void endgameAndFinalWarningsFireExactlyOnceEach() {
        MatchClockRumble cues = new MatchClockRumble();
        cues.start(100);
        int endgame = 0, last = 0;
        for (double t = 100; t <= 100 + MatchClockRumble.TELEOP_SECONDS + 5; t += .125) {
            cues.update(t);
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
        MatchClockRumble cues = new MatchClockRumble();
        cues.start(0);
        cues.update(ENDGAME_AT);
        assertTrue(cues.endgameWarning());
        cues.update(ENDGAME_AT + .02);
        assertFalse(cues.endgameWarning());
    }

    @Test public void lateStartPastBothMarksGivesOnlyTheFinalWarning() {
        MatchClockRumble cues = new MatchClockRumble();
        cues.start(0);
        cues.update(FINAL_AT + 1);
        assertTrue(cues.finalWarning());
        assertFalse(cues.endgameWarning());
        cues.update(FINAL_AT + 2);
        assertFalse(cues.endgameWarning());
        assertFalse(cues.finalWarning());
    }

    @Test public void nothingBeforeStartAndClockIsClamped() {
        MatchClockRumble cues = new MatchClockRumble();
        cues.update(500);
        assertFalse(cues.finalWarning());
        assertFalse(cues.endgameWarning());
        assertEquals(MatchClockRumble.TELEOP_SECONDS, cues.secondsLeft(500), 0);
        cues.start(0);
        assertEquals(MatchClockRumble.TELEOP_SECONDS - 30, cues.secondsLeft(30), 1e-9);
        assertEquals(0, cues.secondsLeft(MatchClockRumble.TELEOP_SECONDS + 10), 0);
    }
}
