package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.junit.Test;

import static org.junit.Assert.*;

/** Stage 1 runs open-loop full power for loading, run-on, override and shots; its encoder is telemetry only. */
public class TransferStage1SpeedTest {
    private final FakeHardwareMap map = new FakeHardwareMap();
    // Fake top speed 2800 ticks/s = 6000 RPM at 28 ticks/rev (goBILDA 5203 Yellow Jacket, 1:1).
    private final FakeMotor stage1 = new FakeMotor(2800, 10, 28);
    private final FakeMotor stage2 = new FakeMotor(2800, 10, 28);
    private final FakeDigitalChannel full = new FakeDigitalChannel();
    private final TransferSubsystem transfer;

    public TransferStage1SpeedTest() {
        map.register("transfer_motor_1", stage1);
        map.register("transfer_motor_2", stage2);
        map.register("distance_sensor", full);
        full.setSimulatedInputState(false);
        transfer = new TransferSubsystem(map);
    }

    @Test public void stage1AlwaysRunsFullPowerOpenLoop() {
        transfer.update(0, false, false);
        assertFullPower(); // Automatic loading.
        assertEquals(0, stage2.getPower(), 0);

        transfer.update(.02, true, false); // Shot feed.
        assertFullPower();
        assertEquals(1, stage2.getPower(), 0);

        full.setSimulatedInputState(true);
        transfer.update(.04, false, false);
        transfer.update(.04 + TransferSubsystem.FULL_CONFIRM_TIME_SECONDS, false, false);
        assertTrue(transfer.isFull());
        assertFullPower(); // Fourth-ball run-on.
        double afterRunOn = .05 + TransferSubsystem.FULL_CONFIRM_TIME_SECONDS + TransferSubsystem.STAGE_1_FULL_RUN_ON_SECONDS;
        transfer.update(afterRunOn, false, false);
        assertEquals(0, stage1.getPower(), 0); // Full: stopped until LOW or a shot.
        transfer.update(afterRunOn + .02, false, false, true, true);
        assertFullPower(); // Intake-forward override past full.
    }

    @Test public void measuredStage1SpeedComesFromItsOwnEncoder() {
        transfer.update(0, false, false);
        for (int i = 1; i <= 100; i++) { stage1.update(.01); transfer.update(i * .01, false, false); }
        assertEquals(6000, transfer.getStage1MeasuredRpm(), 300);
    }

    private void assertFullPower() {
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, stage1.getMode());
        assertEquals(1, stage1.getPower(), 0);
    }
}
