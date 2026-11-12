package org.firstinspires.ftc.teamcode;

import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class PollenNavigatorTest {
    private static final long START = 1_000_000_000L;
    private PollenNavigator navigator;
    private static long at(double seconds) { return START + (long) (seconds * 1e9); }
    private static PollenTarget target(long time, double forwardMm, double rightMm) {
        double pitch = Math.toRadians(15), z = forwardMm * Math.cos(pitch) + 180 * Math.sin(pitch);
        double down = -forwardMm * Math.sin(pitch) + 180 * Math.cos(pitch);
        double x = 160 + 280 * rightMm / z, y = 120 + 280 * down / z;
        return PollenTarget.fromFloorContact(time, x, y - 10, x, y, 150, 280, 280, 160, 120, 180, 15, 0, 0);
    }
    private void update(double t, PollenTarget target, double x, double y, double heading, boolean ready, boolean full) {
        navigator.update(at(t), target, x, y, heading, ready, full);
    }
    private void stopped() {
        assertEquals(PollenNavigator.Motion.STOP, navigator.getMotion());
        assertEquals(0, navigator.getIntakePower(), 0);
    }
    @Before public void start() { navigator = new PollenNavigator(); navigator.start(START, 0, 0); }
    private void pickup(double heading) {
        for (double t : new double[]{0, .07, .15}) update(t, target(at(t), 640, 0), 0, 0, heading, true, false);
        assertEquals(PollenNavigator.State.COLLECTING, navigator.getState());
    }
    @Test public void waitsForHardwareAndBoundsStartupCalibration() {
        navigator = new PollenNavigator();
        update(0, target(START, 1000, 0), 0, 0, 0, true, false); stopped();
        navigator.start(START, Double.NaN, Double.NaN);
        update(0, null, Double.NaN, Double.NaN, Double.NaN, false, false); stopped();
        update(1, target(at(1), 1000, 0), 0, 0, 0, true, false);
        assertEquals(0, navigator.getTotalTravelMm(), 0);
        navigator.start(START, 0, 0);
        update(4.1, null, 0, 0, 0, false, false);
        assertEquals(PollenNavigator.State.FAULT, navigator.getState()); stopped();
    }
    @Test public void searchRequestsClockwiseHoldAtFixedPositionAndIsBounded() {
        update(0, null, 20, 30, 179, true, false);
        assertEquals(PollenNavigator.Motion.HOLD, navigator.getMotion());
        assertEquals(20, navigator.getGoalXMm(), 0); assertEquals(30, navigator.getGoalYMm(), 0);
        assertEquals(89, navigator.getGoalHeadingDeg(), 0);
        update(.2, null, 20, 30, 88, true, false);
        assertEquals(PollenNavigator.State.TIMED_OUT, navigator.getState()); stopped();
        start(); update(0, null, 0, 0, 0, true, false);
        update(4.1, null, 0, 0, 0, true, false); stopped();
    }
    @Test public void alignsWithoutTranslationAndTransformsRightBearingIntoCcwFrame() {
        update(0, target(START, 700, 350), 100, 200, 90, true, false);
        assertEquals(PollenNavigator.Motion.HOLD, navigator.getMotion());
        assertEquals(100, navigator.getGoalXMm(), 1e-8); assertEquals(200, navigator.getGoalYMm(), 1e-8);
        assertEquals(90 - Math.toDegrees(Math.atan2(350,700)), navigator.getGoalHeadingDeg(), 1e-8);
        update(.1, target(at(.1), 700, -350), 100, 200, 90, true, false);
        assertTrue(navigator.getGoalHeadingDeg() > 90);
    }
    @Test public void alignedApproachRequestsAnAbsolutePedroGoalWithPickupStandOff() {
        update(0, target(START, 1200, 0), 100, 200, 90, true, false);
        assertEquals(PollenNavigator.Motion.FOLLOW, navigator.getMotion());
        assertEquals(100, navigator.getGoalXMm(), 1e-8);
        assertEquals(200 + 1200 - PollenNavigator.PICKUP_START_FORWARD_MM, navigator.getGoalYMm(), 1e-8);
    }
    @Test public void staleMissingFutureAndOutOfOrderTargetsStopAndCanReacquire() {
        for (int mode = 0; mode < 4; mode++) {
            start(); update(0, target(START, 1000, 0), 0, 0, 0, true, false);
            PollenTarget invalid = mode == 0 ? null : target(mode == 1 ? START : mode == 2 ? at(1) : START - 1, 1000, 0);
            update(.21, invalid, 20, 0, 0, true, false); stopped();
            update(.6, null, 20, 0, 0, true, false); stopped();
            assertEquals(PollenNavigator.State.SEARCHING, navigator.getState());
            update(.7, target(at(.7), 900, 0), 20, 0, 0, true, false);
            assertEquals(PollenNavigator.Motion.FOLLOW, navigator.getMotion());
        }
    }
    @Test public void distinctCloseFramesCommitOneFixedBlindPickupGoal() {
        PollenTarget single = target(START, 640, 0);
        for (double t : new double[]{0, .07, .15}) update(t, single, 0, 0, 179, true, false);
        assertEquals(PollenNavigator.State.APPROACHING, navigator.getState());
        start(); pickup(179);
        double x = navigator.getGoalXMm(), y = navigator.getGoalYMm();
        update(.2, null, 20, 0, -179, true, false);
        assertEquals(PollenNavigator.Motion.FOLLOW, navigator.getMotion());
        assertEquals(x, navigator.getGoalXMm(), 0); assertEquals(y, navigator.getGoalYMm(), 0);
        assertEquals(179, navigator.getGoalHeadingDeg(), 0);
        update(.4, null, -451, 0, -179, true, false);
        assertEquals(PollenNavigator.State.SETTLING, navigator.getState());
        assertEquals(PollenNavigator.Motion.STOP, navigator.getMotion()); assertEquals(1, navigator.getPickupAttempts());
        update(.81, null, -451, 0, -179, true, false); stopped();
    }
    @Test public void targetLossResetsCloseConfirmationAndPickupHasATimeLimit() {
        update(0, target(START, 640, 0), 0, 0, 0, true, false);
        update(.05, null, 0, 0, 0, true, false);
        update(.15, target(at(.15), 640, 0), 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.APPROACHING, navigator.getState());
        start(); pickup(0); update(1.06, null, 0, 0, 0, true, false);
        assertEquals(PollenNavigator.State.SETTLING, navigator.getState());
        assertEquals(PollenNavigator.Motion.STOP, navigator.getMotion());
    }
    @Test public void fullQueueHardwareFailureInvalidPoseAndStopCancelPickup() {
        for (int mode = 0; mode < 4; mode++) {
            start(); pickup(0);
            if (mode == 3) navigator.stop(at(.2));
            else update(.2, null, mode == 2 ? Double.NaN : 20, 0, 0, mode != 1, mode == 0);
            stopped(); assertTrue(navigator.isFinished());
            update(.3, target(at(.3), 1000, 0), 0, 0, 0, true, false); stopped();
        }
    }
    @Test public void totalTravelAutoDeadlineAndApproachDeadlineAreBounded() {
        update(0, target(START, 1000, 0), 0, 0, 0, true, false);
        update(.1, target(at(.1), 1000, 0), 1300, 0, 0, true, false);
        update(.2, target(at(.2), 1000, 0), 0, 0, 0, true, false); stopped();
        assertTrue(navigator.isFinished());
        start(); update(28.1, null, 0, 0, 0, true, false); stopped();
        start(); update(0, target(START, 800, 0), 0, 0, 0, true, false);
        update(7.1, target(at(7.1), 800, 0), 0, 0, 0, true, false); stopped();
        assertEquals(PollenNavigator.State.TIMED_OUT, navigator.getState());
    }
}
