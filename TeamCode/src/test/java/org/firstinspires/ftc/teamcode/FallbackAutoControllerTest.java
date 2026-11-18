package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.FallbackAutoController.Phase;

public class FallbackAutoControllerTest {
    private static final double M = .0254;

    /** Fresh READY Pinpoint sample; inches and degrees in, meters and radians stored. */
    private static MotionSample pose(double now, double xIn, double yIn, double headingDeg) {
        return new MotionSample(now, xIn * M, yIn * M, Math.toRadians(headingDeg), 0, 0, 0, 0, 1, true, false, "READY");
    }

    /** Leave-only controller already in LEAVE with its origin at (0, 0, headingDeg). */
    private static FallbackAutoController leaving(double forward, double left, double headingDeg) {
        FallbackAutoController auto = new FallbackAutoController(0, false, forward, left);
        auto.start(0);
        auto.update(0, pose(0, 0, 0, headingDeg), false, false);
        assertEquals(Phase.LEAVE, auto.phase());
        return auto;
    }

    @Test public void leaveDrivesForwardSlowsNearTheTargetAndStops() {
        FallbackAutoController auto = leaving(24, 0, 0);
        auto.update(.125, pose(.125, 0, 0, 0), false, false);
        assertEquals(FallbackAutoController.MAX_DRIVE_POWER, auto.axial(), 1e-9);
        assertEquals(0, auto.lateral(), 1e-9);
        assertEquals(0, auto.yaw(), 1e-9);
        assertFalse(auto.flywheelOn());
        assertFalse(auto.feedRequested());

        auto.update(1, pose(1, 21, 0, 0), false, false); // 3 in to go: proportional, above the minimum.
        assertEquals(FallbackAutoController.KP_POWER_PER_INCH * 3, auto.axial(), 1e-9);
        auto.update(1.125, pose(1.125, 23.5, 0, 0), false, false); // 0.5 in: inside tolerance.
        assertEquals(Phase.DONE, auto.phase());
        assertEquals(0, auto.axial(), 1e-9);
        auto.update(2, pose(2, 23.5, 0, 0), false, false);
        assertEquals(0, auto.axial(), 1e-9);
        assertEquals("Leave complete", auto.status());
    }

    @Test public void minimumPowerKeepsTheRobotMovingJustOutsideTolerance() {
        FallbackAutoController auto = leaving(24, 0, 0);
        auto.update(.125, pose(.125, 22.75, 0, 0), false, false); // 1.25 in to go.
        assertEquals(FallbackAutoController.MIN_DRIVE_POWER, auto.axial(), 1e-9);
    }

    @Test public void targetIsRelativeToTheStartHeadingAndSignsMatchTheDrivetrain() {
        // Started facing field +Y: "forward 24" is field +Y, and the robot drives straight forward.
        FallbackAutoController auto = leaving(24, 0, 90);
        auto.update(.125, pose(.125, 0, 0, 90), false, false);
        assertEquals(FallbackAutoController.MAX_DRIVE_POWER, auto.axial(), 1e-9);
        assertEquals(0, auto.lateral(), 1e-9);

        // "Left" is Pinpoint +Y; MecanumDrivetrain's +lateral is right, so a left target needs negative lateral.
        auto = leaving(0, 12, 0);
        auto.update(.125, pose(.125, 0, 0, 0), false, false);
        assertEquals(0, auto.axial(), 1e-9);
        assertEquals(-FallbackAutoController.MAX_DRIVE_POWER, auto.lateral(), 1e-9);
    }

    @Test public void headingHoldTurnsBackClockwiseAfterACounterClockwiseDrift() {
        FallbackAutoController auto = leaving(24, 0, 0);
        auto.update(.125, pose(.125, 0, 0, 10), false, false);
        // +yaw is clockwise in MecanumDrivetrain.
        assertEquals(FallbackAutoController.KP_YAW_POWER_PER_DEG * 10, auto.yaw(), 1e-9);
        auto.update(.25, pose(.25, 0, 0, -60), false, false);
        assertEquals(-FallbackAutoController.MAX_YAW_POWER, auto.yaw(), 1e-9);
    }

    @Test public void movingAwayFromTheTargetFaultsAndStops() {
        FallbackAutoController auto = leaving(24, 0, 0);
        auto.update(.125, pose(.125, 0, 0, 0), false, false);
        auto.update(.25, pose(.25, -FallbackAutoController.WRONG_WAY_INCHES - 1, 0, 0), false, false);
        assertEquals(Phase.FAULT, auto.phase());
        assertEquals(0, auto.axial(), 1e-9);
        assertTrue(auto.status(), auto.status().contains("AWAY"));
    }

    @Test public void lostOrStalePoseStopsThenFaults() {
        FallbackAutoController auto = leaving(24, 0, 0);
        auto.update(.125, null, false, false);
        assertEquals(Phase.LEAVE, auto.phase());
        assertEquals(0, auto.axial(), 1e-9);
        auto.update(.25, pose(0, 0, 0, 0), false, false); // 0.25 s old: stale.
        assertEquals(0, auto.axial(), 1e-9);
        auto.update(.125 + FallbackAutoController.POSE_LOSS_SECONDS, null, false, false);
        assertEquals(Phase.FAULT, auto.phase());
    }

    @Test public void notReadyPinpointNeverMovesTheRobot() {
        FallbackAutoController auto = new FallbackAutoController(0, false, 24, 0);
        auto.start(0);
        for (double t = 0; t < FallbackAutoController.POSE_LOSS_SECONDS + .5; t += .125) {
            auto.update(t, new MotionSample(t, 0, 0, 0, 0, 0, 0, 0, 1, false, false, "CALIBRATING"), false, false);
            assertEquals(0, auto.axial(), 1e-9);
        }
        assertEquals(Phase.FAULT, auto.phase());
    }

    @Test public void leaveTimesOutAndStops() {
        FallbackAutoController auto = leaving(24, 0, 0);
        auto.update(FallbackAutoController.LEAVE_TIMEOUT_SECONDS, pose(FallbackAutoController.LEAVE_TIMEOUT_SECONDS, 10, 0, 0), false, false);
        assertEquals(Phase.DONE, auto.phase());
        assertTrue(auto.status(), auto.status().contains("TIMEOUT"));
        assertEquals(0, auto.axial(), 1e-9);
    }

    @Test public void zeroLeaveDistanceFinishesImmediately() {
        FallbackAutoController auto = leaving(0, 0, 0);
        auto.update(.125, pose(.125, 0, 0, 0), false, false);
        assertEquals(Phase.DONE, auto.phase());
    }

    @Test public void startDelayHoldsStillThenLeaves() {
        FallbackAutoController auto = new FallbackAutoController(3, false, 24, 0);
        auto.start(10);
        auto.update(12.875, pose(12.875, 0, 0, 0), false, false);
        assertEquals(Phase.START_DELAY, auto.phase());
        assertFalse(auto.flywheelOn());
        auto.update(13, pose(13, 0, 0, 0), false, false);
        assertEquals(Phase.LEAVE, auto.phase());
        auto.update(13.125, pose(13.125, 0, 0, 0), false, false);
        assertEquals(FallbackAutoController.MAX_DRIVE_POWER, auto.axial(), 1e-9);
    }

    @Test public void preloadShotSpinsUpFeedsThenLeaves() {
        FallbackAutoController auto = new FallbackAutoController(1, true, 24, 0);
        auto.start(0);
        auto.update(.5, pose(.5, 0, 0, 0), true, true);
        assertEquals(Phase.START_DELAY, auto.phase());
        assertTrue("flywheel spins up during the delay", auto.flywheelOn());
        assertFalse(auto.feedRequested());
        auto.update(1, pose(1, 0, 0, 0), true, false);
        assertEquals(Phase.SPIN_UP, auto.phase());
        auto.update(1.5, pose(1.5, 0, 0, 0), true, false);
        assertTrue(auto.flywheelOn());
        assertFalse(auto.feedRequested());
        assertEquals(0, auto.axial(), 1e-9);

        auto.update(2, pose(2, 0, 0, 0), true, true);
        assertEquals(Phase.FEED, auto.phase());
        assertTrue(auto.feedRequested());
        auto.update(2 + FallbackAutoController.FEED_SECONDS - .125, pose(4.375, 0, 0, 0), true, false);
        assertTrue("feed is a timed hold, not readiness-gated", auto.feedRequested());
        assertEquals(0, auto.axial(), 1e-9);
        auto.update(2 + FallbackAutoController.FEED_SECONDS, pose(4.5, 0, 0, 0), true, true);
        assertEquals(Phase.LEAVE, auto.phase());
        assertFalse(auto.feedRequested());
        assertFalse(auto.flywheelOn());
        assertFalse(auto.fedBeforeReady());
        auto.update(4.625, pose(4.625, 0, 0, 0), true, true);
        assertEquals(FallbackAutoController.MAX_DRIVE_POWER, auto.axial(), 1e-9);
    }

    @Test public void spinUpTimeoutFeedsAnywayOnlyWhenTheTurretIsHomed() {
        FallbackAutoController homed = new FallbackAutoController(0, true, 24, 0);
        homed.start(0);
        homed.update(0, pose(0, 0, 0, 0), true, false);
        homed.update(FallbackAutoController.SPIN_UP_TIMEOUT_SECONDS, pose(3, 0, 0, 0), true, false);
        assertEquals(Phase.FEED, homed.phase());
        assertTrue(homed.feedRequested());
        assertTrue(homed.fedBeforeReady());

        FallbackAutoController unhomed = new FallbackAutoController(0, true, 24, 0);
        unhomed.start(0);
        for (double t = 0; t <= FallbackAutoController.SPIN_UP_TIMEOUT_SECONDS; t += .125) {
            unhomed.update(t, pose(t, 0, 0, 0), false, true);
            assertFalse("never feed without a turret home", unhomed.feedRequested());
        }
        assertEquals(Phase.LEAVE, unhomed.phase());
        assertTrue(unhomed.status(), unhomed.status().contains("SKIPPED"));
    }

    @Test public void deadlineStopsEverything() {
        FallbackAutoController auto = new FallbackAutoController(0, true, 24, 0);
        auto.start(0);
        auto.update(0, pose(0, 0, 0, 0), true, false);
        auto.update(FallbackAutoController.MAX_AUTONOMOUS_SECONDS, pose(29, 0, 0, 0), true, true);
        assertEquals(Phase.DONE, auto.phase());
        assertFalse(auto.flywheelOn());
        assertFalse(auto.feedRequested());
    }

    @Test public void nothingHappensBeforeStart() {
        FallbackAutoController auto = new FallbackAutoController(0, true, 24, 0);
        auto.update(0, pose(0, 0, 0, 0), true, true);
        assertFalse(auto.flywheelOn());
        assertEquals(0, auto.axial(), 1e-9);
    }

    @Test public void initDelayStepsAreClampedAndSettingsValidated() {
        assertEquals(0, FallbackAutoController.stepDelay(0, -1), 0);
        assertEquals(2, FallbackAutoController.stepDelay(1, 1), 0);
        assertEquals(FallbackAutoController.MAX_START_DELAY_SECONDS,
                FallbackAutoController.stepDelay(FallbackAutoController.MAX_START_DELAY_SECONDS, 1), 0);
        try { new FallbackAutoController(-1, false, 24, 0); fail(); } catch (IllegalArgumentException expected) { }
        try { new FallbackAutoController(0, false, Double.NaN, 0); fail(); } catch (IllegalArgumentException expected) { }
    }
}
