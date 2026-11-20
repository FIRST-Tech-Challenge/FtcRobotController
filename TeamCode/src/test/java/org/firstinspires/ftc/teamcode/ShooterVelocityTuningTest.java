package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShooterVelocityTuningTest {
    private static final double KS = .05, KV = .0004, KP = .001;
    private static double fusion(double target, double measured, double volts) {
        return ShooterVelocityTuning.fusionPower(target, measured, volts, KS, KV, KP);
    }
    @Test public void fusionUsesFullPowerOnlyAboveEightyTicksPerSecondError() {
        double target = 1300; // motor ticks/s, about 2786 flywheel RPM
        assertEquals(80, ShooterVelocityTuning.FUSION_BANG_BANG_ERROR_TICKS_PER_SECOND, 0);
        // Slow by more than 80 ticks/s: full power in the commanded direction.
        for (double measured : new double[] {-100, 0, 900, 1219.9}) {
            assertTrue(ShooterVelocityTuning.fusionUsesBangBang(target, measured));
            assertEquals(1, fusion(target, measured, 12), 0);
            assertEquals(-1, fusion(-target, -measured, 12), 0);
        }
        // Fast by more than 80 ticks/s: coast at zero power, never reverse-power braking.
        for (double measured : new double[] {1380.1, 1600, 2500}) {
            assertTrue(ShooterVelocityTuning.fusionUsesBangBang(target, measured));
            assertEquals(0, fusion(target, measured, 12), 0);
            assertEquals(0, fusion(-target, -measured, 12), 0);
        }
        // Exactly 80 ticks/s of error is still inside the feedforward band.
        for (double measured : new double[] {1220, 1260, 1300, 1340, 1380}) {
            assertFalse(ShooterVelocityTuning.fusionUsesBangBang(target, measured));
            double expected = KS + KV * target + KP * (target - measured);
            assertEquals(expected, fusion(target, measured, 12), 1e-12);
            assertEquals(-expected, fusion(-target, -measured, 12), 1e-12);
            assertTrue(expected > 0 && expected < 1);
        }
    }
    @Test public void fusionFeedforwardCompensatesBatteryAndStaysWithinMotorLimits() {
        double atTarget = KS + KV * 1300;
        assertEquals(atTarget * 12 / 10, fusion(1300, 1300, 10), 1e-12);
        assertEquals(atTarget * 12 / 14, fusion(1300, 1300, 14), 1e-12);
        // An invalid battery reading uses the 12 V reference instead of scaling the command.
        for (double volts : new double[] {0, 8.9, 16.1, Double.NaN, Double.POSITIVE_INFINITY})
            assertEquals(atTarget, fusion(1300, 1300, volts), 1e-12);
        assertEquals(1, ShooterVelocityTuning.fusionPower(2500, 2500, 12, .3, .002, 0), 0);
        assertEquals(-1, ShooterVelocityTuning.fusionPower(-2500, -2500, 12, .3, .002, 0), 0);
        // Inside the band a correction larger than the feedforward stops at zero; it never reverses.
        assertEquals(0, ShooterVelocityTuning.fusionPower(40, 120, 12, 0, 0, .001), 0);
        assertEquals(0, ShooterVelocityTuning.fusionPower(-40, -120, 12, 0, 0, .001), 0);
    }
    @Test public void fusionStopAndInvalidInputsNeverPowerMotor() {
        // A zero target coasts: it never reverse-brakes a spinning flywheel.
        assertEquals(0, fusion(0, 2000, 12), 0);
        assertEquals(0, fusion(0, -2000, 12), 0);
        assertFalse(ShooterVelocityTuning.fusionUsesBangBang(0, 2000));
        for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertEquals(0, fusion(1300, invalid, 12), 0);
            assertEquals(0, fusion(invalid, 0, 12), 0);
            assertFalse(ShooterVelocityTuning.fusionUsesBangBang(1300, invalid));
            assertFalse(ShooterVelocityTuning.fusionUsesBangBang(invalid, 0));
            assertEquals(0, ShooterVelocityTuning.fusionPower(1300, 1300, 12, invalid, KV, KP), 0);
            assertEquals(0, ShooterVelocityTuning.fusionPower(1300, 1300, 12, KS, invalid, KP), 0);
            assertEquals(0, ShooterVelocityTuning.fusionPower(1300, 1300, 12, KS, KV, invalid), 0);
        }
    }
    @Test public void initialFusionGainsReproduceTheInstalledSpeedObservation() {
        // Full power at 6100 RPM and 14 V: the only speed measurement available. kS/kP are estimates.
        double observedTicksPerSecond = 6100.0 / 60 * 28;
        assertEquals(1, (ShooterVelocityTuning.INITIAL_FUSION_KS
                + ShooterVelocityTuning.INITIAL_FUSION_KV * observedTicksPerSecond) * 12 / 14, 1e-12);
        assertTrue(ShooterVelocityTuning.INITIAL_FUSION_KS <= ShooterVelocityTuning.MAX_FUSION_KS);
        assertTrue(ShooterVelocityTuning.INITIAL_FUSION_KV <= ShooterVelocityTuning.MAX_FUSION_KV);
        assertTrue(ShooterVelocityTuning.INITIAL_FUSION_KP <= ShooterVelocityTuning.MAX_FUSION_KP);
    }
    @Test public void feedforwardMatchesConfiguredMotorSpeedAndCompensatesBatterySag() {
        double base = 32767.0 / (6100.0 / 60 * 28) * 14.0 / 12;
        assertEquals(base, ShooterVelocityTuning.feedforward(12, 1), 1e-9);
        assertEquals(base * 12 / 10, ShooterVelocityTuning.feedforward(10, 1), 1e-9);
        assertEquals(base * 12 / 14, ShooterVelocityTuning.feedforward(14, 1), 1e-9);
        assertEquals(base * 1.1, ShooterVelocityTuning.feedforward(12, 1.1), 1e-9);
    }
    @Test public void invalidVoltageFallsBackAndTuningAdjustmentsAreBounded() {
        double base = ShooterVelocityTuning.BASE_F;
        for (double v : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY})
            assertEquals(base, ShooterVelocityTuning.feedforward(v, 1), 1e-9);
        assertEquals(base * 1.5, ShooterVelocityTuning.feedforward(12, 10), 1e-9);
        assertEquals(base * .5, ShooterVelocityTuning.feedforward(12, -1), 1e-9);
        assertEquals(base, ShooterVelocityTuning.feedforward(12, Double.NaN), 1e-9);
    }
    @Test public void fullSpeedTargetUsesTheConfiguredMaximumWithoutReserve() {
        assertEquals(1, ShooterVelocityTuning.estimatedFreeSpeedUsage(6100, 14), 1e-9);
        assertEquals(6000.0 / 6100, ShooterVelocityTuning.estimatedFreeSpeedUsage(6000, 14), 1e-9);
        assertEquals(6000, ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM, 0);
        assertTrue(ShooterVelocityTuning.estimatedFreeSpeedUsage(ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM, 10) > 1);
        assertEquals(0, ShooterVelocityTuning.estimatedFreeSpeedUsage(0, 12), 0);
    }
}
