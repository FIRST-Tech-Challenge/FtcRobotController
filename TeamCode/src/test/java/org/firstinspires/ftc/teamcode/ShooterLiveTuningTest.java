package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.junit.Test;
import static org.junit.Assert.*;

/** Hub command regressions only: fake hardware does not establish loaded flywheel tuning. */
public class ShooterLiveTuningTest {
    private static final class RecordingMotor extends FakeMotor {
        int writes;
        double p, i, d, f;
        RecordingMotor() { super(2590, 8, 28); }
        @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
            writes++;
            this.p = p; this.i = i; this.d = d; this.f = f;
            super.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    private TurretSubsystem turret(RecordingMotor motor) { return turret(motor, 10); }
    private TurretSubsystem turret(FakeMotor motor, double batteryVolts) {
        FakeHardwareMap map = new FakeHardwareMap();
        map.register("turret_motor", motor);
        map.register("turret_encoder", new FakeMotor(2800, 8, 28));
        for (String name : new String[] {"turret_servo_1", "turret_servo_2", "turret_pitch"})
            map.register(name, new FakeServo());
        map.register("Control Hub", new FakeVoltageSensor(batteryVolts));
        return new TurretSubsystem(map, false, () -> 0);
    }

    /** Synthetic plant only: full power is 6100 RPM, with no friction, ball load or voltage sag. */
    @Test public void fusionSpinsUpAtFullPowerThenHoldsInsideFeedforwardBandOnFakeFlywheel() {
        FakeMotor motor = new FakeMotor(6100.0 / 60 * 28, 10, 28);
        TurretSubsystem turret = turret(motor, 14); // 6100 RPM was observed at 14 V.
        assertEquals(TurretSubsystem.ShooterController.PIDF, turret.getShooterController());
        turret.setShooterRpm(2800);
        turret.setShooterController(TurretSubsystem.ShooterController.BANG_BANG_FUSION);
        // Switching stops the command; it must be requested again.
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.getMode());
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, motor.getZeroPowerBehavior());
        assertEquals(0, motor.getPower(), 0);
        assertEquals(0, turret.getTargetShooterRpm(), 0);

        double dt = .02, minimumSettledPower = 1, maximumSettledPower = 0;
        for (int n = 0; n < 400; n++) {
            turret.update(n * dt);
            turret.setShooterRpm(2800);
            if (n == 0) {
                assertTrue(turret.isShooterFusionBangBang());
                assertEquals(1, motor.getPower(), 0);
            }
            assertTrue("reverse power at loop " + n, motor.getPower() >= 0);
            if (n >= 300) {
                assertFalse("left the feedforward band at loop " + n, turret.isShooterFusionBangBang());
                minimumSettledPower = Math.min(minimumSettledPower, motor.getPower());
                maximumSettledPower = Math.max(maximumSettledPower, motor.getPower());
            }
            motor.update(dt);
        }
        turret.update(400 * dt);
        assertEquals(2800, turret.getShooterCurrentRpm(), TurretSubsystem.SHOOTER_READY_TOLERANCE_RPM);
        // Holding is a steady partial-power command, not full-power cycling.
        assertTrue(minimumSettledPower > 0 && maximumSettledPower < 1);
        assertEquals(minimumSettledPower, maximumSettledPower, .02);

        // A target more than 80 ticks/s (171 RPM) above the wheel returns to full power.
        turret.setShooterRpm(3400);
        assertTrue(turret.isShooterFusionBangBang());
        assertEquals(1, motor.getPower(), 0);
        // A target that far below the wheel coasts down to it; no reverse power.
        turret.setShooterRpm(2200);
        assertTrue(turret.isShooterFusionBangBang());
        assertEquals(0, motor.getPower(), 0);
        // Stopping coasts as well.
        turret.stopShooter();
        assertEquals(0, motor.getPower(), 0);
        assertFalse(turret.isShooterFusionBangBang());
        turret.setShooterController(TurretSubsystem.ShooterController.PIDF);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode());
        turret.stop();
    }

    @Test public void fusionGainsAreBoundedAndInvalidValuesKeepThePriorSetting() {
        TurretSubsystem turret = turret(new RecordingMotor());
        assertEquals(ShooterVelocityTuning.INITIAL_FUSION_KS, turret.getShooterFusionKs(), 0);
        assertEquals(ShooterVelocityTuning.INITIAL_FUSION_KV, turret.getShooterFusionKv(), 0);
        assertEquals(ShooterVelocityTuning.INITIAL_FUSION_KP, turret.getShooterFusionKp(), 0);
        turret.setShooterFusionTuning(.08, .0005, .002);
        turret.setShooterFusionTuning(Double.NaN, .0004, .001);
        turret.setShooterFusionTuning(.05, Double.POSITIVE_INFINITY, .001);
        turret.setShooterFusionTuning(.05, .0004, Double.NEGATIVE_INFINITY);
        assertEquals(.08, turret.getShooterFusionKs(), 0);
        assertEquals(.0005, turret.getShooterFusionKv(), 0);
        assertEquals(.002, turret.getShooterFusionKp(), 0);
        turret.setShooterFusionTuning(5, 5, 5);
        assertEquals(ShooterVelocityTuning.MAX_FUSION_KS, turret.getShooterFusionKs(), 0);
        assertEquals(ShooterVelocityTuning.MAX_FUSION_KV, turret.getShooterFusionKv(), 0);
        assertEquals(ShooterVelocityTuning.MAX_FUSION_KP, turret.getShooterFusionKp(), 0);
        turret.setShooterFusionTuning(-1, -1, -1);
        assertEquals(0, turret.getShooterFusionKs(), 0);
        assertEquals(0, turret.getShooterFusionKv(), 0);
        assertEquals(0, turret.getShooterFusionKp(), 0);
        turret.stop();
    }

    @Test public void liveGainsReachHubWithoutRestartingVelocityOrRewritingUnchangedSettings() {
        RecordingMotor motor = new RecordingMotor();
        TurretSubsystem turret = turret(motor);
        turret.setShooterRpm(2800);
        int initialWrites = motor.writes;
        turret.setShooterVelocityTuning(45, .05, 1, 1.1);
        assertEquals(initialWrites + 1, motor.writes);
        assertEquals(45, motor.p, 0);
        assertEquals(.05, motor.i, 0);
        assertEquals(1, motor.d, 0);
        assertEquals(ShooterVelocityTuning.BASE_F * 12 / 10 * 1.1, motor.f, 1e-9);
        assertEquals(2800, turret.getTargetShooterRpm(), 0);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.getMode());
        for (int n = 0; n < 100; n++) turret.setShooterVelocityTuning(45, .05, 1, 1.1);
        assertEquals(initialWrites + 1, motor.writes);
        turret.resetShooterVelocityTuning();
        assertEquals(ShooterVelocityTuning.INITIAL_P, motor.p, 0);
        assertEquals(ShooterVelocityTuning.INITIAL_I, motor.i, 0);
        assertEquals(ShooterVelocityTuning.INITIAL_D, motor.d, 0);
        assertEquals(ShooterVelocityTuning.feedforward(10, 1), motor.f, 1e-9);
        turret.stop();
        assertEquals(0, motor.getPower(), 0);
    }

    @Test public void invalidOrOutOfRangeLiveGainsCannotSendInvalidCoefficientsToHub() {
        RecordingMotor motor = new RecordingMotor();
        TurretSubsystem turret = turret(motor);
        turret.setShooterVelocityTuning(500, -1, 500, 10);
        assertEquals(100, motor.p, 0);
        assertEquals(0, motor.i, 0);
        assertEquals(100, motor.d, 0);
        assertEquals(ShooterVelocityTuning.feedforward(10, 1.5), motor.f, 1e-9);
        int writes = motor.writes;
        turret.setShooterVelocityTuning(Double.NaN, 1, 0, 1);
        turret.setShooterVelocityTuning(40, Double.POSITIVE_INFINITY, 0, 1);
        turret.setShooterVelocityTuning(40, 0, Double.NaN, 1);
        turret.setShooterVelocityTuning(40, 0, 0, Double.NEGATIVE_INFINITY);
        assertEquals(writes, motor.writes);
        turret.stop();
    }
}
