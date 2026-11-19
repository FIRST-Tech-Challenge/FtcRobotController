package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class FiringCoordinatorTest {
    @Test public void configuredCycleWaitsPointOneSecondsAfterGateCommandAndFeedsTwoTenths() {
        FiringCoordinator f = ShootingConfiguration.firing();
        TransferController transfer = new TransferController();
        transfer.update(0, false, false, false);
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);

        update(f, 1, true, true, true);
        double opened = 1 + f.stableSeconds + .01;
        update(f, opened, true, true, true);
        assertTrue(f.gateOpen());
        // Both stages wait until 0.10 seconds after the open command.
        update(f, opened + .099, true, true, true);
        transfer.update(opened + .099, false, f.feeding(), true);
        assertTrue(f.gateOpen());
        assertFalse(f.feeding());
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);

        double feeding = opened + .101;
        update(f, feeding, true, true, true);
        transfer.update(feeding, true, f.feeding(), true);
        assertTrue(f.feeding());
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(1, transfer.stage2(), 0);
        // The gate and both stages stay active through the 0.20-second feed window.
        update(f, feeding + .15, true, true, true);
        transfer.update(feeding + .15, true, f.feeding(), true);
        assertTrue(f.gateOpen());
        assertTrue(f.feeding());
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(1, transfer.stage2(), 0);
        update(f, feeding + f.feedWindowSeconds - .01, true, true, true);
        assertTrue(f.feeding());

        double recovery = feeding + f.feedWindowSeconds + .01;
        update(f, recovery, true, true, true);
        transfer.update(recovery, false, f.feeding(), true);
        assertEquals(FiringCoordinator.State.RECOVERY, f.state());
        assertFalse(f.gateOpen());
        assertTrue(f.closedGateStage1());
        assertEquals(0, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
        // No recovery pause: the next update goes straight to the readiness wait, gate still closed.
        update(f, recovery + .001, true, true, true);
        assertEquals(FiringCoordinator.State.PREPARING, f.state());
        assertTrue(f.closedGateStage1());

        // Releasing the shot request restores stage 1 loading, with stage 2 off.
        f.update(recovery + .002, false, false, false, false, true, true, true, "");
        assertFalse(f.closedGateStage1());
        transfer.update(recovery + .002, false, f.feeding(), false);
        assertEquals(1, transfer.stage1(), 0);
        assertEquals(0, transfer.stage2(), 0);
    }

    private FiringCoordinator f() { return new FiringCoordinator(.2,.1,.15,.3,.1); }
    private void update(FiringCoordinator f,double now,boolean valid,boolean tight,boolean loose) {
        f.update(now,true,false,false,false,valid,tight,loose,"Invalid synthetic solution");
    }
    @Test public void stableReadinessGateTravelFeedingAndRpmRecovery() {
        FiringCoordinator f=f(); update(f,0,true,true,true); assertFalse(f.gateOpen());
        update(f,.11,true,true,true); assertTrue(f.gateOpen()); assertFalse(f.feeding());
        update(f,.32,true,true,true); assertTrue(f.feeding()); assertEquals(1,f.feedAttempts());
        update(f,.36,true,false,false); assertFalse(f.feeding()); assertFalse(f.gateOpen());
        assertEquals(FiringCoordinator.State.RECOVERY,f.state()); assertTrue(f.ballMayBeCommitted());
        update(f,.7,true,true,true); update(f,.71,true,true,true); update(f,.82,true,true,true);
        assertEquals(FiringCoordinator.State.GATE_OPENING,f.state());
    }
    @Test public void staleOrInvalidImmediatelyStopsFurtherFeedingAndInhibitWins() {
        FiringCoordinator f=f(); update(f,0,true,true,true); update(f,.11,true,true,true); update(f,.32,true,true,true);
        update(f,.33,false,true,true); assertFalse(f.feeding()); assertEquals(FiringCoordinator.State.INHIBITED,f.state());
        f.update(.4,true,true,true,false,true,true,true,""); assertFalse(f.gateOpen());
        assertEquals("Operator feed inhibit",f.reason());
    }
    @Test public void overridePriorityAndHysteresisAreDeterministic() {
        FiringCoordinator f=f();
        f.update(0,true,false,false,true,true,true,true,""); assertFalse(f.gateOpen());
        f.update(.1,true,true,false,true,false,false,false,"invalid"); assertTrue(f.gateOpen());
        f.update(.31,true,true,false,true,false,false,false,"invalid"); assertTrue(f.feeding());
        f.stop(.32); assertFalse(f.feeding()); assertFalse(f.gateOpen());
        f=f(); update(f,0,true,true,true); update(f,.11,true,true,true); update(f,.15,true,false,true);
        assertTrue(f.gateOpen()); // Entry tolerance differs from retention tolerance.
    }
    @Test public void currentPoseSolutionMayFeedDuringMotionAndStopsOnInvalidTarget() {
        FiringCoordinator f=f(); boolean sawFeed=false;
        for(int i=0;i<100;i++) {
            double now=1+i*.01;
            ShotPlanner.Solution s=planner(mount()).solve(now,pose(now,0,0,0,0,.4,0),goal(2,0),profile());
            assertTrue(s.reason,s.valid); assertEquals(0,s.yaw,0);
            boolean angleReady = ShotReadiness.angleReady(2.9);
            update(f,now,i<60 && s.valid,angleReady,angleReady); sawFeed|=f.feeding();
            if(i>=60) assertFalse(f.feeding());
        }
        assertTrue(sawFeed); assertTrue(f.feedAttempts()>0);
    }
    @Test public void manualFeedStaysContinuousAndAutoMustRequalifyAfterOverride() {
        FiringCoordinator f=ShootingConfiguration.firing(); f.update(0,true,true,false,false,true,true,true,"");
        assertTrue(f.gateOpen());
        f.update(.099,true,true,false,false,true,true,true,""); assertFalse(f.feeding());
        f.update(.101,true,true,false,false,true,true,true,""); assertTrue(f.feeding());
        f.update(1,true,true,false,false,true,true,true,""); assertTrue(f.feeding());
        f.update(1.01,true,false,false,false,true,true,true,""); assertFalse(f.feeding()); assertFalse(f.gateOpen());
        f.update(1.01+f.stableSeconds+.01,true,false,false,false,true,true,true,""); assertTrue(f.gateOpen());
    }
}
