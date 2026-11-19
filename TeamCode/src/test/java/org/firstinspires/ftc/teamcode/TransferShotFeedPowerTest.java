package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.junit.Test;

import static org.junit.Assert.*;

/** Shots always run both transfer stages at full power; only loading while intaking is stall-guarded. */
public class TransferShotFeedPowerTest {
    /** Stage 1 that never turns (0 RPM) while drawing stall current. */
    private static final class StalledMotor extends FakeMotor {
        StalledMotor() { super(2800, 10, 28); }
        @Override public double getCurrent(CurrentUnit unit) { return 9.0; }
    }

    private final FakeHardwareMap map = new FakeHardwareMap();
    private final StalledMotor stage1 = new StalledMotor();
    private final FakeMotor stage2 = new FakeMotor(2800, 10, 28);
    private final TransferSubsystem transfer;

    public TransferShotFeedPowerTest() {
        FakeDigitalChannel full = new FakeDigitalChannel();
        full.setSimulatedInputState(false);
        map.register("transfer_motor_1", stage1);
        map.register("transfer_motor_2", stage2);
        map.register("distance_sensor", full);
        transfer = new TransferSubsystem(map);
    }

    @Test public void shotFeedKeepsBothStagesAtFullPowerEvenWhenStage1ReadsStalled() {
        transfer.setIntakeRunning(true);
        for (double t = 0; t < 5; t += .02) {
            transfer.update(t, true, false); // Feeding: stage 1 and stage 2 together.
            assertEquals("stage 1 at t=" + t, 1, stage1.getPower(), 0);
            assertEquals("stage 2 at t=" + t, 1, stage2.getPower(), 0);
            assertFalse(transfer.isStage1StallProtected());
        }
    }

    @Test public void bothTransferMotorsUseTheCorrectedDirections() {
        // Both transfers are wired backwards relative to the original build (corrected 2026-11-18).
        assertEquals(DcMotor.Direction.FORWARD, stage1.getDirection());
        assertEquals(DcMotor.Direction.REVERSE, stage2.getDirection());
    }

    @Test public void stallGuardIsInactiveWhileTheIntakeIsNotRunningForward() {
        transfer.setIntakeRunning(false);
        for (double t = 0; t < 10; t += .02) {
            transfer.update(t, false, false, true, true); // Closed-gate / override stage 1 into a jam.
            assertEquals("stage 1 at t=" + t, 1, stage1.getPower(), 0);
            assertFalse(transfer.isStage1StallProtected());
        }
    }

    @Test public void stoppingTheIntakeClearsAStallLockout() {
        transfer.setIntakeRunning(true);
        double t = 0;
        for (; t < 10 && transfer.getStallState() != TransferStallGuard.State.LOCKED_OUT; t += .02)
            transfer.update(t, false, false);
        assertEquals(TransferStallGuard.State.LOCKED_OUT, transfer.getStallState());
        assertEquals(0, stage1.getPower(), 0);

        transfer.setIntakeRunning(false);
        transfer.update(t, false, false);
        assertEquals(TransferStallGuard.State.OK, transfer.getStallState());
        assertEquals(1, stage1.getPower(), 0);
    }

    @Test public void loadingStallCutsStage1ButTheNextShotRestoresFullPower() {
        transfer.setIntakeRunning(true); // The guard only protects stage 1 while intaking.
        double t = 0;
        for (; t < 10 && transfer.getStallState() != TransferStallGuard.State.LOCKED_OUT; t += .02)
            transfer.update(t, false, false); // Loading into a jam.
        assertEquals(TransferStallGuard.State.LOCKED_OUT, transfer.getStallState());
        assertEquals(0, stage1.getPower(), 0);
        assertEquals(0, stage2.getPower(), 0); // Stage 2 stays off while loading.

        transfer.update(t, true, false); // Shot request.
        assertEquals(1, stage1.getPower(), 0);
        assertEquals(1, stage2.getPower(), 0);
        assertFalse(transfer.isStage1StallProtected());
    }

    @Test public void feedWithStage1HeldBackStillRunsStage2AtFullPower() {
        transfer.update(0, true, false, false); // Stage 2 first.
        assertEquals(0, stage1.getPower(), 0);
        assertEquals(1, stage2.getPower(), 0);
        for (double t = .02; t < 3; t += .02) {
            transfer.update(t, true, false, true); // Stage 1 released.
            assertEquals(1, stage1.getPower(), 0);
            assertEquals(1, stage2.getPower(), 0);
        }
    }
}
