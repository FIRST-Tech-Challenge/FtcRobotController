package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class CompetitionFeedControllerTest {
    @Test public void continuousStartsByDefaultAndHeldToggleSelectsOneStaggeredCyclePolicy() {
        CompetitionFeedController c = new CompetitionFeedController();
        FiringCoordinator f = CompetitionFeedController.firing();
        // Robot-tuned staggered timing is shared by competition, autonomous and the bench test.
        assertEquals(0.10, f.gateTravelSeconds, 0);
        assertEquals(0.20, f.feedWindowSeconds, 0);
        assertEquals(0.0, f.recoverySeconds, 0);
        assertEquals(f.gateTravelSeconds, ShootingConfiguration.firing().gateTravelSeconds, 0);
        assertEquals(f.feedWindowSeconds, ShootingConfiguration.firing().feedWindowSeconds, 0);
        assertEquals(f.recoverySeconds, ShootingConfiguration.firing().recoverySeconds, 0);
        step(c, f, 0, false, true, false);
        assertTrue(f.feeding());
        assertTrue(c.feedTransfers(f, false));
        step(c, f, .21, false, true, false);
        assertTrue(c.feedStage1(.21, f));
        step(c, f, 2, false, true, false);
        assertTrue(f.feeding());

        step(c, f, 3, true, true, false);
        assertFalse(c.continuous());
        assertFalse(f.feeding());
        step(c, f, 3.09, true, true, false);
        assertFalse(c.feedStage1(3.09, f));
        assertFalse(c.feedTransfers(f, false));
        step(c, f, 3.11, true, true, false);
        assertTrue(f.feeding());
        assertTrue(c.feedStage1(3.11, f)); // Stage 1 starts with stage 2 once the gate delay has elapsed.
        assertTrue(c.feedTransfers(f, true));
        step(c, f, 3.21, true, true, false);
        assertTrue(c.feedStage1(3.21, f));
        step(c, f, 3.42, true, true, false);
        assertEquals(FiringCoordinator.State.RECOVERY, f.state());
        assertFalse(f.gateOpen());
        // Gate closed after a completed timed feed: only stage 1 runs; stage 2 is off.
        assertFalse(c.feedStage1(3.42, f));
        assertFalse(c.feedTransfers(f, false));
        assertTrue(c.closedGateStage1(f));
        step(c, f, 3.51, true, true, false); // No recovery pause: straight to readiness wait.
        assertFalse(f.gateOpen());
        assertFalse(c.feedTransfers(f, false));
        assertTrue(c.closedGateStage1(f));
        assertEquals(FiringCoordinator.State.PREPARING, f.state());
        step(c, f, 3.52, true, true, false);
        assertTrue(f.gateOpen());
        assertFalse(c.feedTransfers(f, false));
        assertFalse(c.closedGateStage1(f)); // Stage 1 stops again at the next opening.
        step(c, f, 3.63, true, true, false);
        assertTrue(f.feeding());
        assertTrue(c.feedStage1(3.63, f)); // Every staggered shot starts both stages together.
        assertTrue(c.feedTransfers(f, true));
        step(c, f, 3.68, true, true, false);
        assertTrue(c.feedStage1(3.68, f));

        step(c, f, 4.31, false, true, false);
        step(c, f, 4.32, true, true, false);
        assertTrue(c.continuous());
        assertTrue(f.feeding()); // Continuous starts with the gate command.
        step(c, f, 4.53, true, true, false);
        assertTrue(c.feedStage1(4.53, f));

        // Automatic readiness can extend closed-gate feeding beyond the fixed recovery.
        c.updateSelection(5, false, f);
        c.updateSelection(5, true, f);
        f.update(5, true, false, false, false, true, true, true, "");
        f.update(5.16, true, false, false, false, true, true, true, "");
        f.update(5.27, true, false, false, false, true, true, true, "");
        f.update(5.58, true, false, false, false, true, true, true, "");
        f.update(5.94, true, false, false, false, true, false, false, "");
        f.update(6.2, true, false, false, false, true, false, false, "");
        assertFalse(f.gateOpen());
        assertFalse(c.feedTransfers(f, false));
        assertTrue(c.closedGateStage1(f)); // Readiness wait: gate closed, stage 1 still runs.
        f.update(6.3, true, false, false, false, true, true, true, "");
        f.update(6.46, true, false, false, false, true, true, true, "");
        assertTrue(f.gateOpen());
        assertFalse(c.feedTransfers(f, false));
        assertFalse(c.closedGateStage1(f));
        f.update(6.47, true, false, true, false, true, true, true, "");
        assertFalse(c.feedTransfers(f, false));
        assertFalse(c.closedGateStage1(f));
    }

    @Test public void bothModesHonorInhibitReleaseAndAutomaticReadiness() {
        for (boolean staggered : new boolean[]{false, true}) {
            CompetitionFeedController c = new CompetitionFeedController();
            FiringCoordinator f = CompetitionFeedController.firing();
            step(c, f, 0, staggered, true, false);
            step(c, f, .21, false, true, false);
            assertTrue(f.feeding());
            step(c, f, .22, false, true, true);
            assertFalse(f.gateOpen());
            assertFalse(c.feedStage1(.22, f));
            step(c, f, .23, false, false, false);
            assertEquals(FiringCoordinator.State.IDLE, f.state());
            step(c, f, .24, false, true, false);
            assertEquals(!staggered, f.feeding());

            f.update(.25, true, false, false, false, false, false, false, "Uncalibrated");
            assertFalse(f.gateOpen());
            assertFalse(c.feedStage1(.25, f));
            f.update(1, true, false, false, false, true, true, true, "");
            assertFalse(f.gateOpen());
            f.update(1.16, true, false, false, false, true, true, true, "");
            assertTrue(f.gateOpen());
            c.feedStage1(1.16, f);
            f.update(1.37, true, false, false, false, true, true, true, "");
            assertTrue(f.feeding());
            f.update(1.38, true, false, false, false, true, false, false, "RPM dropped");
            assertFalse(f.feeding());
            assertFalse(c.feedStage1(1.38, f));
            assertFalse(c.feedTransfers(f, false));
            assertFalse(c.closedGateStage1(f)); // Tracking-loss recovery never runs closed-gate transfers.
        }
    }

    private void step(CompetitionFeedController c, FiringCoordinator f, double now,
                      boolean toggle, boolean manual, boolean inhibit) {
        c.updateSelection(now, toggle, f);
        // Manual feeding keeps its existing readiness bypass, even in staggered mode.
        f.update(now, false, manual, inhibit, false, false, false, false, "Not ready");
        c.feedStage1(now, f);
    }
}
