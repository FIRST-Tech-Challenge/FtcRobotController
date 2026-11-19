package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.junit.Test;

import static org.junit.Assert.*;

/**
 * Encoder-port routing and SDK direction regressions; no physical yaw calibration claim.
 * Wiring: the turret Through-Bore encoder is on intake_motor's encoder input (Expansion Hub port 1;
 * the intake has no encoder) and transfer_motor_1 has its own motor encoder on port 2.
 */
public class TurretEncoderWiringTest {
    private final FakeHardwareMap map = new FakeHardwareMap();
    private final FakeMotor intake = new FakeMotor(2590, 10, 28);
    private final FakeMotor transfer = new FakeMotor(2800, 10, 28);

    private void configureHardware() {
        map.register("intake_motor", intake);
        map.register("transfer_motor_1", transfer);
        map.register("transfer_motor_2", new FakeMotor(2590, 10, 28));
        map.register("turret_motor", new FakeMotor(2590, 10, 28));
        map.register("distance_sensor", new FakeDigitalChannel());
        for (String name : new String[]{"gate_servo", "turret_servo_1", "turret_servo_2", "turret_pitch"})
            map.register(name, new FakeServo());
        intake.setEncoderMeasuringAnotherMechanism(6144, 3072);
    }

    @Test public void turretUsesIntakeChannelAndPreservesIntakeMotorConfiguration() {
        configureHardware();
        // A legacy alias must not redirect feedback to transfer stage 1's own motor encoder.
        map.register("turret_encoder", transfer);
        new IntakeSubsystem(map).setIntakePower(.6);
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        turret.update(0);
        assertYawFeedback(turret);
        assertEquals(DcMotor.Direction.FORWARD, intake.getDirection());
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, intake.getMode());
        assertEquals(.6, intake.getPower(), 0);
    }

    @Test public void reversedSharedChannelDirectionCannotFlipYaw() {
        configureHardware();
        intake.setDirection(DcMotor.Direction.REVERSE);
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        turret.update(0);
        assertEquals(-6144, intake.getCurrentPosition());
        assertYawFeedback(turret);
    }

    @Test public void laterIntakeInitializationCannotFlipYawOrResetItsZero() {
        configureHardware();
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        turret.update(0);
        assertYawFeedback(turret);
        turret.zeroEncoderAtCurrentPosition();
        new IntakeSubsystem(map);
        intake.setEncoderMeasuringAnotherMechanism(12288, 3072);
        turret.update(.02);
        assertYawFeedback(turret);
    }

    @Test public void transferLoadingNeverTouchesTurretFeedback() {
        configureHardware();
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        TransferSubsystem stage = new TransferSubsystem(map);
        stage.update(0, false, false);
        assertEquals(1, transfer.getPower(), 0);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, intake.getMode());
        turret.update(0);
        assertYawFeedback(turret);
    }

    @Test public void standaloneTurretEncoderAliasStillWorksWithoutIntakeHardware() {
        configureHardware();
        map.remove(null, "intake_motor", intake);
        map.register("turret_encoder", intake);
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        turret.update(0);
        assertYawFeedback(turret);
    }

    private void assertYawFeedback(TurretSubsystem turret) {
        assertEquals(6144, turret.getEncoderTicks());
        assertEquals(100, turret.getEncoderAngleDegrees(), 1e-9);
        assertEquals(50, turret.getEncoderVelocityDegPerSec(), 1e-9);
    }
}
