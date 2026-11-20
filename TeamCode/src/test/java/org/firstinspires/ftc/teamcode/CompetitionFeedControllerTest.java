package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class CompetitionFeedControllerTest {
    @Test public void competitionFeedIsContinuousOnlyWithNoStaggeredTimingBlockers() {
        CompetitionFeedController c = new CompetitionFeedController();
        FiringCoordinator f = CompetitionFeedController.firing();
        assertTrue(c.continuous());
        // Shared timing values stay as tuned on the robot (2026-11-15).
        assertEquals(0.10, f.gateTravelSeconds, 0);
        assertEquals(0.20, f.feedWindowSeconds, 0);
        assertEquals(0.0, f.recoverySeconds, 0);
        assertEquals(f.gateTravelSeconds, ShootingConfiguration.firing().gateTravelSeconds, 0);
        assertEquals(f.feedWindowSeconds, ShootingConfiguration.firing().feedWindowSeconds, 0);
        assertEquals(f.recoverySeconds, ShootingConfiguration.firing().recoverySeconds, 0);
        // Manual feed: gate and transfer start together and no timed window ever ends the feed.
        for (int i = 0; i <= 300; i++) {
            double now = i * .01;
            step(c, f, now, true, false);
            assertTrue(f.feeding());
            assertTrue(f.continuous());
            assertTrue(c.feedStage1(now, f));
            assertTrue(c.feedTransfers(f, false));
            assertFalse(c.closedGateStage1(f));
        }
    }

    @Test public void continuousHonorsInhibitReleaseAndAutomaticReadiness() {
        CompetitionFeedController c = new CompetitionFeedController();
        FiringCoordinator f = CompetitionFeedController.firing();
        step(c, f, 0, true, false);
        step(c, f, .21, true, false);
        assertTrue(f.feeding());
        step(c, f, .22, true, true);
        assertFalse(f.gateOpen());
        assertFalse(c.feedStage1(.22, f));
        step(c, f, .23, false, false);
        assertEquals(FiringCoordinator.State.IDLE, f.state());
        step(c, f, .24, true, false);
        assertTrue(f.feeding());

        // Automatic shots still need valid tracking held READY_STABLE_SECONDS, then feed with no gate delay.
        c.updateSelection(.25, f);
        f.update(.25, true, false, false, false, false, false, false, "Uncalibrated");
        assertFalse(f.gateOpen());
        assertFalse(c.feedStage1(.25, f));
        f.update(1, true, false, false, false, true, true, true, "");
        assertFalse(f.gateOpen());
        f.update(1.16, true, false, false, false, true, true, true, "");
        assertTrue(f.feeding());
        assertTrue(c.feedTransfers(f, false));
        f.update(5, true, false, false, false, true, true, true, "");
        assertTrue(f.feeding()); // No timed window ends a continuous automatic feed.
        f.update(5.01, true, false, false, false, true, false, false, "RPM dropped");
        assertFalse(f.feeding());
        assertFalse(c.feedStage1(5.01, f));
        assertFalse(c.feedTransfers(f, false));
        assertFalse(c.closedGateStage1(f)); // Tracking-loss recovery never runs closed-gate transfers.
    }

    private void step(CompetitionFeedController c, FiringCoordinator f, double now, boolean manual, boolean inhibit) {
        c.updateSelection(now, f);
        // Manual feeding bypasses shot readiness.
        f.update(now, false, manual, inhibit, false, false, false, false, "Not ready");
        c.feedStage1(now, f);
    }
}
