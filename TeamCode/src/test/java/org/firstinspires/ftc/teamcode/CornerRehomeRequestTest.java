package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class CornerRehomeRequestTest {
    @Test public void settledPressAppliesImmediatelyInEitherPhase() {
        for (boolean mayWait : new boolean[]{true, false}) {
            CornerRehomeRequest r = new CornerRehomeRequest();
            assertEquals(CornerRehomeRequest.NOT_REQUESTED, r.status());
            assertTrue(r.press(true, mayWait));
            assertEquals(CornerRehomeRequest.APPLIED, r.status());
            assertFalse(r.waiting());
            assertFalse(r.needsAttention());
            assertFalse(r.poll(true)); // Applied once; nothing left to apply.
        }
    }

    @Test public void initPressDuringCalibrationWaitsAndAppliesExactlyOnceWhenSettled() {
        CornerRehomeRequest r = new CornerRehomeRequest();
        assertFalse(r.press(false, true));
        assertTrue(r.waiting());
        assertTrue(r.needsAttention());
        assertTrue(r.status().startsWith("WAITING"));
        assertFalse(r.poll(false));
        assertFalse(r.poll(false));
        assertTrue(r.poll(true));
        assertEquals(CornerRehomeRequest.APPLIED, r.status());
        assertFalse(r.needsAttention());
        assertFalse(r.poll(true));
    }

    @Test public void startDiscardsAWaitingPressButNotAnAppliedOne() {
        CornerRehomeRequest waiting = new CornerRehomeRequest();
        waiting.press(false, true);
        waiting.cancel("Not applied before START");
        assertFalse(waiting.waiting());
        assertTrue(waiting.needsAttention());
        assertEquals("Not applied before START", waiting.status());
        assertFalse(waiting.poll(true)); // Never applied after START, even once settled.

        CornerRehomeRequest applied = new CornerRehomeRequest();
        applied.press(true, true);
        applied.cancel("Not applied before START");
        assertEquals(CornerRehomeRequest.APPLIED, applied.status());
        assertFalse(applied.needsAttention());
    }

    @Test public void activeUnsettledPressIsRefusedAndNeverQueued() {
        CornerRehomeRequest r = new CornerRehomeRequest();
        assertFalse(r.press(false, false));
        assertFalse(r.waiting());
        assertTrue(r.needsAttention());
        assertTrue(r.status().startsWith("REFUSED"));
        assertFalse(r.poll(true)); // The robot may have moved: a refused press must not apply later.
        assertTrue(r.press(true, false)); // A new press once settled applies.
        assertFalse(r.needsAttention());
    }
}
