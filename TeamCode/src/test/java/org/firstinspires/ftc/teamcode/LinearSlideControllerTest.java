package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.*;

public class LinearSlideControllerTest {
    private static final double DT = 0.02;
    // Independent synthetic plant: full power = rated Axon MAX+ speed, no load or lag.
    private static final double PLANT_DEG_PER_SECOND_AT_FULL = 60.0 / 0.115;

    /** Synthetic slide: integrates power into servo-output angle; reports the wrapped analog angle. */
    private static final class Plant {
        double angleDeg, now;
        boolean jammed;
        int direction = 1;
        double raw() { return ((angleDeg % 360) + 360) % 360; }
        void step(double power) {
            if (!jammed) angleDeg += direction * power * PLANT_DEG_PER_SECOND_AT_FULL * DT;
            now += DT;
        }
    }

    private static double run(LinearSlideController c, Plant p, int ticks) {
        double power = 0;
        for (int i = 0; i < ticks; i++) { power = c.update(p.now, p.raw()); p.step(power); }
        return power;
    }

    @Test public void unwrapsAcrossZeroInBothDirections() {
        LinearSlideController c = new LinearSlideController(false);
        c.zero(350);
        c.observe(10);   // +20 across 360 -> 0
        assertEquals(20, c.getPositionDeg(), 1e-9);
        c.observe(340);  // -30 across 0 -> 360
        assertEquals(-10, c.getPositionDeg(), 1e-9);
        for (double raw = 0; raw <= 720; raw += 30) c.observe(raw % 360);
        assertEquals(-10 + 20 + 720, c.getPositionDeg(), 1e-9);
    }

    @Test public void invertedEncoderCountsUpWhenAngleFalls() {
        LinearSlideController c = new LinearSlideController(true);
        c.zero(100);
        c.observe(60);
        assertEquals(40, c.getPositionDeg(), 1e-9);
    }

    @Test public void measuredReadingsGiveMeasuredTravelWithFallingAngle() {
        // The real slide: whatever angle INIT reads is fully down (45.6 deg raw when measured), and the angle
        // falls through the 360/0 wrap as the slide rises about 1.94 turns.
        LinearSlideController c = new LinearSlideController(LinearSlideSubsystem.ENCODER_INVERTED);
        c.zero(45.6);
        double travelled = 0;
        while (travelled + 10 < LinearSlideController.MAX_TRAVEL_DEG) {
            travelled += 10;
            c.observe(((45.6 - travelled) % 360 + 360) % 360);
        }
        c.observe(((45.6 - LinearSlideController.MAX_TRAVEL_DEG) % 360 + 360) % 360);
        assertEquals(LinearSlideController.MAX_TRAVEL_DEG, c.getPositionDeg(), 1e-6);
    }

    @Test public void toggleRaisesOverSeveralTurnsThenLowers() {
        LinearSlideController c = new LinearSlideController(false, 900, 10);
        Plant p = new Plant(); p.angleDeg = 200;
        c.zero(p.raw());
        assertEquals(0, run(c, p, 5), 0);
        assertEquals(LinearSlideController.State.DOWN, c.getState());

        c.toggle(p.now);
        assertTrue(c.update(p.now, p.raw()) > 0);
        run(c, p, 200);
        assertEquals(LinearSlideController.State.UP, c.getState());
        assertEquals(900, c.getPositionDeg(), LinearSlideController.TOLERANCE_DEG);
        assertEquals(0, c.getPower(), 0);

        c.toggle(p.now);
        run(c, p, 200);
        assertEquals(LinearSlideController.State.DOWN, c.getState());
        assertEquals(0, c.getPositionDeg(), LinearSlideController.TOLERANCE_DEG);
        assertEquals(0, c.getPower(), 0);
    }

    @Test public void jamFaultsWithZeroPowerAndNextToggleRetries() {
        LinearSlideController c = new LinearSlideController(false, 900, 10);
        Plant p = new Plant(); p.jammed = true;
        c.zero(p.raw());
        c.raise(p.now);
        run(c, p, (int) Math.ceil(LinearSlideController.STALL_WINDOW_SECONDS / DT) + 2);
        assertEquals(LinearSlideController.State.FAULT, c.getState());
        assertTrue(c.getFaultReason().contains("stalled"));
        assertEquals(0, run(c, p, 10), 0);

        p.jammed = false;
        c.toggle(p.now);   // retry: now lowering (already at 0)
        run(c, p, 5);
        assertEquals(LinearSlideController.State.DOWN, c.getState());
    }

    @Test public void wrongDirectionFaults() {
        LinearSlideController c = new LinearSlideController(false, 900, 10);
        Plant p = new Plant(); p.direction = -1;
        c.zero(p.raw());
        c.raise(p.now);
        run(c, p, 50);
        assertEquals(LinearSlideController.State.FAULT, c.getState());
        assertTrue(c.getFaultReason().contains("wrong way"));
        assertEquals(0, c.getPower(), 0);
    }

    @Test public void overallTimeoutFaults() {
        LinearSlideController c = new LinearSlideController(false, 900, 0.3);
        Plant p = new Plant();
        c.zero(p.raw());
        c.raise(p.now);
        run(c, p, 30);
        assertEquals(LinearSlideController.State.FAULT, c.getState());
        assertTrue(c.getFaultReason().contains("not there"));
    }

    @Test public void softLimitsBlockDrivingPastEitherEnd() {
        LinearSlideController c = new LinearSlideController(false, 180, 10);
        c.zero(0);
        c.raise(0);
        c.observe(0);
        // Settled-hold correction above the top must not drive further up.
        for (double raw = 0; raw <= 200; raw += 20) c.observe(raw);
        assertTrue(c.getPositionDeg() > 180 + LinearSlideController.LIMIT_MARGIN_DEG);
        assertTrue(c.update(1, 200) <= 0);
    }

    @Test public void manualReleaseStaysUnpoweredUntilNextCommand() {
        LinearSlideController c = new LinearSlideController(false, 900, 10);
        Plant p = new Plant();
        c.zero(p.raw());
        c.setManualPower(0.5, p.now);
        run(c, p, 20);
        assertEquals(LinearSlideController.State.MANUAL, c.getState());
        assertTrue(c.getPositionDeg() > 50);
        c.setManualPower(Double.NaN, p.now);
        assertEquals(0, run(c, p, 20), 0);   // does not drive back to 0
        c.lower(p.now);
        assertTrue(c.update(p.now, p.raw()) < 0);
    }

    @Test public void measuredTravelIsUnderOneTurnAndTimeoutCoversIt() {
        assertTrue(LinearSlideController.TRAVEL_MEASURED);
        // 2026-11-20: up is 1.75 turns + 70 deg CCW viewed from the servo face, with the analog angle falling.
        assertEquals(700, LinearSlideController.MAX_TRAVEL_DEG, 1e-9);
        assertTrue(LinearSlideSubsystem.ENCODER_INVERTED);
        // Robot test 2026-11-20: REVERSE drove the slide down into its bottom stop.
        assertEquals(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD, LinearSlideSubsystem.SERVO_DIRECTION);
        assertTrue(LinearSlideController.MOVE_TIMEOUT_SECONDS
                > LinearSlideController.MAX_TRAVEL_DEG / AxonMaxPlus.DEGREES_PER_SECOND_AT_6V);
    }
}
