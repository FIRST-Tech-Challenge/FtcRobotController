package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

/** Volley sequencing: aim -> ready -> feed -> finish, and both timeout outcomes. Hardware-free. */
public class PlannerVolleyTest {
    private final PlannerVolley v = new PlannerVolley();

    @Test public void readyFeedsForTheFeedTimeThenFinishes() {
        v.start(10, 2, 3, PlannerVolley.TimeoutAction.FIRE, true);
        assertEquals(PlannerVolley.Phase.AIMING, v.phase());
        assertTrue("aiming requests TeleOp RT + LT", v.automaticFeed());
        assertFalse(v.manualFeed());
        v.update(10.5, false, true);
        assertEquals(PlannerVolley.Phase.AIMING, v.phase());
        v.update(11, true, true);   // firing coordinator started feeding (ready + stable)
        assertEquals(PlannerVolley.Phase.FEEDING, v.phase());
        assertTrue("still the readiness-gated feed", v.automaticFeed());
        v.update(12.99, false, true);   // a tracking drop pauses feeding; the feed time keeps running
        assertTrue(v.active());
        v.update(13, false, true);
        assertEquals(PlannerVolley.Phase.DONE, v.phase());
        assertFalse(v.automaticFeed()); assertFalse(v.manualFeed());
        assertTrue(v.finished());
    }

    @Test public void timeoutFiresAnywayWithAHomedTurret() {
        v.start(0, 1, 3, PlannerVolley.TimeoutAction.FIRE, true);
        v.update(2.99, false, true);
        assertEquals(PlannerVolley.Phase.AIMING, v.phase());
        v.update(3, false, true);
        assertEquals(PlannerVolley.Phase.FEEDING, v.phase());
        assertTrue("TeleOp GP1 A: readiness bypassed", v.manualFeed());
        assertFalse(v.automaticFeed());
        assertEquals("fed before READY (timeout)", v.note());
        v.update(4, false, true);
        assertEquals(PlannerVolley.Phase.DONE, v.phase());
    }

    @Test public void timeoutNeverFiresWithAnUnhomedTurret() {
        v.start(0, 1, 3, PlannerVolley.TimeoutAction.FIRE, true);
        v.update(3, false, false);
        assertEquals(PlannerVolley.Phase.SKIPPED, v.phase());
        assertFalse(v.manualFeed());
        assertEquals("SKIPPED: turret not homed", v.note());
    }

    @Test public void timeoutSkipEndsWithoutFeeding() {
        v.start(0, 1, 2, PlannerVolley.TimeoutAction.SKIP, true);
        v.update(2, false, true);
        assertEquals(PlannerVolley.Phase.SKIPPED, v.phase());
        assertFalse(v.automaticFeed()); assertFalse(v.manualFeed());
    }

    @Test public void manualShotFeedsAtOnce() {
        v.start(5, 1.5, 3, PlannerVolley.TimeoutAction.FIRE, false);
        assertEquals(PlannerVolley.Phase.FEEDING, v.phase());
        assertTrue(v.manualFeed());
        v.update(6.49, false, false);
        assertTrue(v.active());
        v.update(6.5, false, false);
        assertEquals(PlannerVolley.Phase.DONE, v.phase());
    }

    @Test public void cancelStopsAnActiveVolleyOnly() {
        v.cancel("idle");
        assertEquals(PlannerVolley.Phase.IDLE, v.phase());
        v.start(0, 1, 1, PlannerVolley.TimeoutAction.FIRE, true);
        v.cancel("stop all");
        assertEquals(PlannerVolley.Phase.SKIPPED, v.phase());
        assertFalse(v.automaticFeed());
    }

    @Test(expected = IllegalArgumentException.class) public void rejectsZeroFeedTime() {
        v.start(0, 0, 1, PlannerVolley.TimeoutAction.FIRE, true);
    }

    @Test(expected = IllegalArgumentException.class) public void rejectsNonFiniteTimeout() {
        v.start(0, 1, Double.NaN, PlannerVolley.TimeoutAction.FIRE, true);
    }
}
