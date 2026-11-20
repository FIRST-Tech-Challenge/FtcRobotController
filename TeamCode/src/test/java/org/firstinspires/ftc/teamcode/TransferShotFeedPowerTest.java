package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * The transfer (both linked motors) always runs at the commanded full power for shots and loading;
 * there is no stall limit or safety cut-off.
 */
public class TransferShotFeedPowerTest {
    /** A linked motor that never turns (0 RPM) while drawing stall current. */
    private static final class StalledMotor extends FakeMotor {
        StalledMotor() { super(2800, 10, 28); }
        @Override public double getCurrent(CurrentUnit unit) { return 9.0; }
    }

    private final FakeHardwareMap map = new FakeHardwareMap();
    private final StalledMotor stage1 = new StalledMotor();
    private final FakeMotor motor2 = new FakeMotor(2800, 10, 28);
    private final TransferSubsystem transfer;

    public TransferShotFeedPowerTest() {
        FakeDigitalChannel full = new FakeDigitalChannel();
        full.setSimulatedInputState(false);
        map.register("transfer_motor_1", stage1);
        map.register("transfer_motor_2", motor2);
        map.register("distance_sensor", full);
        transfer = new TransferSubsystem(map);
    }

    @Test public void shotFeedKeepsBothMotorsAtFullPowerEvenWhenStage1ReadsStalled() {
        for (double t = 0; t < 5; t += .02) {
            transfer.update(t, true, false); // Feeding: both linked motors together.
            assertEquals("motor 1 at t=" + t, 1, stage1.getPower(), 0);
            assertEquals("motor 2 at t=" + t, 1, motor2.getPower(), 0);
        }
    }

    @Test public void bothTransferMotorsUseTheCorrectedDirections() {
        // Motor 2 is mechanically linked to stage 1 (2026-11-18), so both must turn the same way.
        assertEquals(DcMotor.Direction.FORWARD, stage1.getDirection());
        assertEquals(DcMotor.Direction.FORWARD, motor2.getDirection());
        assertEquals(stage1.getMode(), motor2.getMode());
        assertEquals(stage1.getZeroPowerBehavior(), motor2.getZeroPowerBehavior());
    }

    @Test public void loadingIntoAStallIsNeverCut() {
        // No stall limit (removed 2026-11-19): 0 RPM at stall current on both motors keeps full power.
        FakeHardwareMap stalled = new FakeHardwareMap();
        StalledMotor first = new StalledMotor(), second = new StalledMotor();
        FakeDigitalChannel full = new FakeDigitalChannel();
        full.setSimulatedInputState(false);
        stalled.register("transfer_motor_1", first);
        stalled.register("transfer_motor_2", second);
        stalled.register("distance_sensor", full);
        TransferSubsystem t2 = new TransferSubsystem(stalled);
        for (double t = 0; t < 10; t += .02) {
            t2.update(t, false, false);
            assertEquals("motor 1 at t=" + t, 1, first.getPower(), 0);
            assertEquals("motor 2 at t=" + t, 1, second.getPower(), 0);
            t2.update(t + .01, false, false, true, true); // Closed-gate / override stage 1 into a jam.
            assertEquals(1, first.getPower(), 0);
            assertEquals(1, second.getPower(), 0);
        }
    }

    @Test public void feedWithStage1HeldBackHoldsBothLinkedMotors() {
        transfer.update(0, true, false, false); // feedStage1=false: no separate stage left to run.
        assertEquals(0, stage1.getPower(), 0);
        assertEquals(0, motor2.getPower(), 0);
        for (double t = .02; t < 3; t += .02) {
            transfer.update(t, true, false, true); // Stage 1 released.
            assertEquals(1, stage1.getPower(), 0);
            assertEquals(1, motor2.getPower(), 0);
        }
    }
}
