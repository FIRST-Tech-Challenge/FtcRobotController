package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class TurretControllerTest {
    @Test public void competitionReleaseDiscardsQueuedTravelAndDoesNotChaseEncoderNoise() {
        TurretControl c = new TurretControl();
        c.setEncoderCorrectionEnabled(true);
        double target = c.competitionManualYawTarget(0, 0, 1, .1, false);
        assertTrue(target < -30);
        assertEquals(-12, c.competitionManualYawTarget(target, -12, 0, .02, false), 0);
        for (int i = 0; i < 100; i++)
            assertEquals(-12, c.competitionManualYawTarget(-12, -12 + i % 3, 0, .02, false), 0);
        c.competitionManualYawTarget(80, -12, 0, .02, true);
        assertEquals(45, c.competitionManualYawTarget(80, 45, 0, .02, false), 0);
        c.competitionManualYawTarget(80, 45, 0, .02, true);
        double resumed = c.competitionManualYawTarget(80, 45, 1, .02, false);
        assertTrue(resumed < 45 && resumed > 45 - TurretControl.MAX_YAW_DEG_PER_SECOND * .02);
    }
    @Test public void competitionSteadySpeedIsLinearRatedAndSmoothedAcrossLoopRates() {
        for (double stick : new double[] { -.5, .5, 1 }) {
            TurretControl a = new TurretControl(), b = new TurretControl();
            double x = 0, y = 0, previous = 0;
            for (int i = 0; i < 10; i++) {
                previous = x;
                x = a.competitionManualYawTarget(x, 0, stick, .02, false);
                y = b.competitionManualYawTarget(y, 0, stick, .01, false);
                y = b.competitionManualYawTarget(y, 0, stick, .01, false);
                assertEquals(x, y, 1e-9);
                assertTrue(Math.abs(x - previous) <= TurretControl.MAX_YAW_DEG_PER_SECOND * .02);
                if (i == 0) assertTrue(Math.abs(x) < Math.abs(stick) * TurretControl.MAX_YAW_DEG_PER_SECOND * .02);
            }
            assertEquals(-stick * TurretControl.MAX_YAW_DEG_PER_SECOND, (x - previous) / .02, .001);
        }
    }
    @Test public void defaultServoHoldNeverBacktracksForEncoderErrorAfterRelease() {
        TurretControl c = new TurretControl();
        assertFalse(c.isEncoderCorrectionEnabled());
        c.updateYawCommand(0, 0, 0, 0, true, 0);
        double target = c.manualYawTarget(0, 1, .02, false);
        for (int i = 1; i <= 1000; i++) {
            double measured = target + (i % 2 == 0 ? 20 : -20);
            double command = c.updateYawCommand(i * .02, target, measured, 0, true, 0);
            if (i > 5) assertEquals(target, command, 0);
            assertEquals(target, c.manualYawTarget(target, 0, .02, false), 0);
            assertFalse(c.hasAppliedEncoderCorrection());
        }
    }
    @Test public void reportedMountingReversesServoMappingAndKeepsBoundsOrdered() {
        TurretControl c = new TurretControl();
        double rightYaw = c.manualYawTarget(0, 1, .02, false);
        double leftYaw = c.manualYawTarget(0, -1, .02, false);
        assertTrue(rightYaw < 0);
        assertTrue(leftYaw > 0);
        assertTrue(TurretSubsystem.angleDegreesToRotationServoPos(rightYaw) > .5);
        assertTrue(TurretSubsystem.angleDegreesToRotationServoPos(leftYaw) < .5);
        assertEquals(.5, TurretSubsystem.clampRotationServoPosition(.5), 0);
        assertTrue(TurretSubsystem.clampRotationServoPosition(0) > 0);
        assertTrue(TurretSubsystem.clampRotationServoPosition(1) < 1);
    }

    @Test public void encoderNoiseAndErrorsInsideMarginNeverChangeTheCommand() {
        TurretControl c = new TurretControl();
        c.setEncoderCorrectionEnabled(true);
        for (int i = 0; i < 1000; i++) {
            double measured = 90 + (i % 2 == 0 ? 3 : -3);
            double command = c.updateYawCommand(i * .02, 90, measured, 0, true, 0);
            if (i > 5) assertEquals(90, command, 0);
            assertFalse(c.hasAppliedEncoderCorrection());
        }
    }
    @Test public void smallOdometryTargetNoiseDoesNotPreventQualifiedCorrection() {
        TurretControl c = new TurretControl(); c.setEncoderCorrectionEnabled(true);
        for (int i = 0; i <= 15; i++) c.updateYawCommand(i * .02, 90 + i * .01, 84, 0, true, 0);
        assertTrue(c.isEncoderCorrectionActive());
        assertTrue(c.getEncoderCorrectionDegrees() >= 6);
    }
    @Test public void correctionHasThreeDegreeEntryAndOnePointFiveDegreeExitHysteresis() {
        TurretControl c = new TurretControl(); c.setEncoderCorrectionEnabled(true);
        c.updateYawCommand(0, 90, 87, 0, true, 0);
        c.updateYawCommand(.3, 90, 87, 0, true, 0);
        assertFalse(c.isEncoderCorrectionActive()); // Exactly 3 deg does not start correction.
        c.updateYawCommand(.4, 90, 86, 0, true, 0);
        c.updateYawCommand(.66, 90, 86, 0, true, 0);
        assertTrue(c.isEncoderCorrectionActive()); assertEquals(4, c.getEncoderCorrectionDegrees(), 0);
        c.updateYawCommand(.7, 90, 88, 0, true, 0);
        c.updateYawCommand(.96, 90, 88, 0, true, 0);
        assertTrue(c.isEncoderCorrectionActive()); assertEquals(6, c.getEncoderCorrectionDegrees(), 0);
        c.updateYawCommand(.97, 90, 88.5, 0, true, 0);
        assertTrue(c.isEncoderCorrectionActive()); // Exactly 1.5 deg retains correction.
        c.updateYawCommand(.98, 90, 88.6, 0, true, 0);
        assertFalse(c.isEncoderCorrectionActive());
        for (int i = 0; i < 100; i++) {
            c.updateYawCommand(1 + i * .02, 90, 88 + (i % 2) * .5, 0, true, 0);
            assertEquals(6, c.getEncoderCorrectionDegrees(), 0);
            assertFalse(c.isEncoderCorrectionActive());
        }
        c.updateYawCommand(4, 90, 86, 0, true, 0);
        c.updateYawCommand(4.26, 90, 86, 0, true, 0);
        assertTrue(c.isEncoderCorrectionActive());
        c.updateYawCommand(5, 40, 40, 0, true, 0);
        assertFalse(c.isEncoderCorrectionActive()); assertEquals(0, c.getEncoderCorrectionDegrees(), 0);
    }
    @Test public void motionErrorReversalAndUnhomedStatePreventCorrection() {
        TurretControl c = new TurretControl();
        c.setEncoderCorrectionEnabled(true);
        c.updateYawCommand(0, 90, 84, 0, true, 0);
        c.updateYawCommand(.3, 90, 84, 10, true, 0);
        c.updateYawCommand(.6, 90, 84, 0, true, 0);
        c.updateYawCommand(.8, 90, 96, 0, true, 0);
        c.updateYawCommand(1, 90, 96, 0, true, 0);
        assertFalse(c.hasAppliedEncoderCorrection());
        c.updateYawCommand(1.1, 90, 96, 0, true, 0);
        assertEquals(-6, c.getEncoderCorrectionDegrees(), 0);
        c.reset();
        for (int i = 0; i < 100; i++) c.updateYawCommand(i * .02, 90, 0, 0, false, 0);
        assertFalse(c.hasAppliedEncoderCorrection());
    }
    @Test public void encoderAdjustmentIsBoundedByCorrectionAndMechanicalLimits() {
        for (double target : new double[] { -175, -90, 90, 175 }) {
            TurretControl c = new TurretControl();
            c.setEncoderCorrectionEnabled(true);
            double measured = target - Math.copySign(100, target);
            c.updateYawCommand(0, target, measured, 0, true, 0);
            c.updateYawCommand(.3, target, measured, 0, true, 0);
            assertTrue(c.hasAppliedEncoderCorrection());
            assertTrue(Math.abs(c.getEncoderCorrectionDegrees()) <= 25);
            // Persistent stall/error must never accumulate beyond either bound.
            for (int i = 1; i <= 100; i++) {
                double command = c.updateYawCommand(.3 + i * .3, target, measured, 0, true, 0);
                assertTrue(Math.abs(c.getEncoderCorrectionDegrees()) <= 25);
                assertTrue(command >= -175 && command <= 175);
            }
        }
    }
    @Test public void commandEdgesAreOnlyLightlySmoothedThenHoldExactly() {
        for (int hz : new int[] { 50, 100 }) {
            TurretControl c = new TurretControl();
            c.updateYawCommand(0, 0, 0);
            double first = c.updateYawCommand(1.0 / hz, 90, 0);
            assertTrue(first > 90 * .85);
            assertTrue(first < 90);
            for (int i = 2; i <= hz; i++) c.updateYawCommand(i / (double) hz, 90, 0);
            // No feedback correction or continuing command changes during a prolonged hold.
            for (int i = hz + 1; i <= hz * 20; i++) {
                assertEquals(90, c.updateYawCommand(i / (double) hz, 90, 0), 0);
            }
            assertTrue(c.updateYawCommand(20 + 1.0 / hz, -90, 0) < -65);
        }
    }
    @Test public void smoothingIsIndependentOfLoopFrequencyAndStaysInsideTravelLimits() {
        TurretControl a = new TurretControl(), b = new TurretControl();
        a.updateYawCommand(0, 0, 0); b.updateYawCommand(0, 0, 0);
        b.updateYawCommand(.01, 90, 0);
        assertEquals(a.updateYawCommand(.02, 90, 0), b.updateYawCommand(.02, 90, 0), 1e-8);
        for (int i = 2; i <= 100; i++) {
            double command = a.updateYawCommand(i * .02, 1000, 0);
            assertTrue(command >= -175 && command <= 175);
        }
        assertEquals(175, a.updateYawCommand(3, 1000, 0), 0);
        for (int i = 1; i <= 100; i++) a.updateYawCommand(3 + i * .02, -1000, 0);
        assertEquals(-175, a.updateYawCommand(6, -1000, 0), 0);
        a.reset();
        assertEquals(0, a.updateYawCommand(7, 0, 0), 0);
    }
    @Test public void manualAndAutomaticReleaseHoldTheServoTarget() {
        TurretControl c = new TurretControl();
        double target = c.manualYawTarget(0, 1, .1, false);
        assertEquals(-60 / .115 * .1, target, 1e-8);
        assertEquals(target, c.manualYawTarget(target, 0, .02, false), 0);
        assertEquals(target, c.manualYawTarget(target, .04, .02, false), 0);
        assertEquals(80, c.manualYawTarget(80, 1, .02, true), 0);
        assertEquals(80, c.manualYawTarget(80, 0, .02, false), 0);
        assertEquals(80 - TurretControl.manualDelta(.5, .02),
                c.manualYawTarget(80, .5, .02, false), 1e-8);
    }
    @Test public void manualSpeedAndHoodEstimateKeepTheRatedServoSpeed() {
        for (int hz : new int[] { 50, 100 }) {
            TurretControl c = new TurretControl();
            c.updateYawCommand(0, 0, 15);
            double yaw = 0;
            for (int i = 1; i <= hz; i++) {
                yaw += TurretControl.manualDelta(.5, 1.0 / hz);
                c.updateYawCommand(i / (double) hz, 0, 15);
            }
            assertEquals(30 / .115, yaw, 1e-8);
            assertEquals(0, c.hoodError(15), 0);
        }
        TurretControl c = new TurretControl();
        c.updateYawCommand(0, 0, 15);
        c.updateYawCommand(.03, 0, 15);
        assertEquals(15 - (60 / .115 / 2) * .03, c.hoodError(15), 1e-8);
    }
    @Test public void zeroIsCapturedOnlyAfterTravelAndStableWindowEvenIfStartIsEarly() {
        TurretHomingController h = new TurretHomingController(1, .3, 4, .2);
        h.start(0); h.update(.1, 5000, 80); assertFalse(h.ready());
        h.update(.8, 3000, 48); assertFalse(h.ready());
        h.update(1.01, 100, 1); h.update(1.2, 50, .5); assertFalse(h.ready());
        h.update(1.4, 10, .1); h.update(1.71, 11, .11);
        assertTrue(h.ready()); assertEquals(11, h.zeroTicks());
        h.update(2, 1000, 10); assertEquals(11, h.zeroTicks());
    }
    @Test public void delayedMovementResetsStableWindowAndRehomeInvalidates() {
        TurretHomingController h = new TurretHomingController(1, .3, 4, .2);
        h.start(0); h.update(1, 500, 8); h.update(1.25, 300, 5); h.update(1.5, 20, .2);
        assertFalse(h.ready()); h.update(1.81, 20, .2); assertTrue(h.ready());
        h.start(2); assertFalse(h.ready()); h.update(3, 40, .4); h.update(3.31, 40, .4);
        assertTrue(h.ready()); assertEquals(40, h.zeroTicks()); h.invalidate(); assertFalse(h.ready());
    }
    @Test public void unstableEncoderTimesOutAndStationaryStallIsOnlyAnEstimate() {
        TurretHomingController h = new TurretHomingController(1, .3, 2, .2); h.start(0);
        for (double t = 0; t < 2.2; t += .1) h.update(t, (int)(t * 1000), t * 20);
        assertEquals(TurretHomingController.State.FAULT, h.state()); assertTrue(h.reason().contains("timeout"));
        h.start(3); h.update(4, 8000, 130); h.update(4.31, 8000, 130);
        assertTrue(h.ready()); assertTrue(h.reason().contains("no absolute"));
    }
    @Test public void robotHeadingConventionWrapsAtFrontAndExcludesLeft() {
        assertEquals(90, TurretControl.yawToRobotHeadingDegrees(0), 1e-8);
        assertEquals(0, TurretControl.yawToRobotHeadingDegrees(90), 1e-8);
        assertEquals(180, TurretControl.yawToRobotHeadingDegrees(-90), 1e-8);
        assertEquals(270, TurretControl.yawToRobotHeadingDegrees(180), 1e-8);
        assertEquals(275, TurretControl.yawToRobotHeadingDegrees(175), 1e-8);
        assertEquals(265, TurretControl.yawToRobotHeadingDegrees(-175), 1e-8);
        for (int heading = 0; heading < 360; heading++) {
            double yaw = TurretControl.robotHeadingToYawDegrees(heading);
            assertEquals(heading, TurretControl.yawToRobotHeadingDegrees(yaw), 1e-8);
            assertEquals(heading > 265 && heading < 275, Math.abs(yaw) > 175 + 1e-8);
        }
        assertEquals(2, TurretControl.robotHeadingToYawDegrees(359)
                - TurretControl.robotHeadingToYawDegrees(1), 1e-8);
    }
    /** Home direction is set once in TurretSubsystem; heading telemetry and the shooter mount both read it. */
    @Test public void rightFacingHomeIsOneOffsetSharedByHeadingsAndShooterMount() {
        assertEquals(90, TurretSubsystem.ROTATION_HOME_ROBOT_HEADING_DEG, 0);
        assertEquals(TurretSubsystem.ROTATION_HOME_ROBOT_HEADING_DEG,
                TurretControl.yawToRobotHeadingDegrees(TurretSubsystem.ROTATION_HOME_ANGLE_DEG), 1e-8);
        // CCW mount offset and CW robot heading describe the same direction.
        assertEquals(0, TurretSubsystem.normalizeAngleDegrees(TurretSubsystem.TURRET_ZERO_ROBOT_HEADING_OFFSET_DEG
                + TurretSubsystem.ROTATION_HOME_ROBOT_HEADING_DEG), 1e-8);
        assertEquals(-Math.PI / 2, ShootingConfiguration.MOUNT.zeroYaw, 1e-12);
    }
    @Test public void hoodDownIncreasesPwmAwayFromUpperStopAndUpReturnsHome() {
        assertEquals(15, TurretSubsystem.hoodTravelToLaunchAngleDegrees(0), 0);
        assertEquals(0, TurretSubsystem.hoodTravelToLaunchAngleDegrees(15), 0);
        assertEquals(-15, TurretSubsystem.hoodTravelToLaunchAngleDegrees(30), 0);
        assertEquals(.5 + 16.0 / 355, TurretSubsystem.pitchDegreesToServoPos(0), 1e-9);
        assertEquals(.5 + 46.0 / 355, TurretSubsystem.pitchDegreesToServoPos(15), 1e-9);
        assertEquals(.5 + 76.0 / 355, TurretSubsystem.pitchDegreesToServoPos(30), 1e-9);
        assertEquals(TurretSubsystem.pitchDegreesToServoPos(30), TurretSubsystem.pitchDegreesToServoPos(50), 0);
        assertEquals(TurretSubsystem.PITCH_HOME_SERVO_POS, TurretSubsystem.pitchDegreesToServoPos(-10), 1e-9);
        double home = TurretSubsystem.pitchDegreesToServoPos(0);
        double firstDown = TurretSubsystem.pitchDegreesToServoPos(.5);
        assertTrue(firstDown > home);
        assertTrue(TurretSubsystem.pitchDegreesToServoPos(1) > firstDown);
        assertEquals(home, TurretSubsystem.clampPitchServoPosition(0), 0);
        assertEquals(home, TurretSubsystem.clampPitchServoPosition(.49), 0);
        assertEquals(firstDown, TurretSubsystem.clampPitchServoPosition(firstDown), 0);
        assertEquals(TurretSubsystem.pitchDegreesToServoPos(30), TurretSubsystem.clampPitchServoPosition(1), 0);
    }
    @Test public void cameraBearingWrapsIntoShooterYawFrame() {
        assertEquals(0, ShootingConfiguration.MOUNT.cameraYaw, 0);
        assertTrue(ShootingConfiguration.MOUNT.measured);
        assertEquals(-90, ShotCoordinator.visualTrackingYawDegrees(90, 0, 180), 1e-8);
        assertEquals(90, ShotCoordinator.visualTrackingYawDegrees(-90, 0, 180), 1e-8);
        assertEquals(-100, ShotCoordinator.visualTrackingYawDegrees(90, 10, 180), 1e-8);
        assertEquals(-175, ShotCoordinator.visualTrackingYawDegrees(-170, 12, 0), 1e-8);
    }
    @Test public void visualHiveTrackingUsesClockwiseCameraErrorWithoutWrappingThroughLeft() {
        assertEquals(-10, ShotCoordinator.visualTrackingYawDegrees(0, 10, 0), 1e-8);
        assertEquals(15, ShotCoordinator.visualTrackingYawDegrees(10, 0, 5), 1e-8);
        double desired = ShotCoordinator.visualTrackingYawDegrees(170, -12, 0);
        assertEquals(175, TurretSubsystem.clampYawAngle(desired), 1e-8);
    }

    @Test public void automaticYawHoldsDeadZoneSideAndUnwindsPastHysteresis() {
        double max = TurretSubsystem.ROTATION_MAX_ANGLE_DEG, hyst = TurretSubsystem.ROTATION_WRAP_HYSTERESIS_DEG;
        assertEquals(120, TurretSubsystem.reachableYawDegrees(120, 100), 1e-9);
        assertEquals(170, TurretSubsystem.reachableYawDegrees(530, 100), 1e-9); // Any wrap normalizes.
        // Inside the rear dead zone: hold the limit on the side the turret is already on.
        assertEquals(max, TurretSubsystem.reachableYawDegrees(178, 170), 1e-9);
        assertEquals(max, TurretSubsystem.reachableYawDegrees(-178, 170), 1e-9);
        assertEquals(-max, TurretSubsystem.reachableYawDegrees(178, -170), 1e-9);
        // Just past the zone on the other side: keep holding until the hysteresis margin is cleared.
        assertEquals(max, TurretSubsystem.reachableYawDegrees(-(max - hyst + .5), 174), 1e-9);
        assertEquals(-(max - hyst - .5), TurretSubsystem.reachableYawDegrees(-(max - hyst - .5), 174), 1e-9);
        assertEquals(-max, TurretSubsystem.reachableYawDegrees(max - hyst + .5, -174), 1e-9);
        // Mid-unwind (turret near home) the far-side target stays committed even near the zone.
        assertEquals(-(max - 1), TurretSubsystem.reachableYawDegrees(-(max - 1), 30), 1e-9);
        assertTrue(Double.isNaN(TurretSubsystem.reachableYawDegrees(Double.NaN, 0)));
    }

    @Test public void deadZoneBearingFromTheFrontHalfTakesTheNearerLimit() {
        double max = TurretSubsystem.ROTATION_MAX_ANGLE_DEG;
        // Freshly homed (0 deg) or anywhere in the front half: go to the limit next to the bearing.
        assertEquals(-max, TurretSubsystem.reachableYawDegrees(-178, 0), 1e-9);
        assertEquals(max, TurretSubsystem.reachableYawDegrees(178, 0), 1e-9);
        assertEquals(-max, TurretSubsystem.reachableYawDegrees(-178, 60), 1e-9);
        assertEquals(max, TurretSubsystem.reachableYawDegrees(178, -60), 1e-9);
        assertEquals(-max, TurretSubsystem.reachableYawDegrees(-178, Double.NaN), 1e-9);
        // Mid-unwind toward -172, a bearing that slips into the zone keeps the unwind going instead of reversing.
        assertEquals(-max, TurretSubsystem.reachableYawDegrees(-177, 30), 1e-9);
        assertEquals(max, TurretSubsystem.reachableYawDegrees(177, -30), 1e-9);
        // In the rear half the turret still holds the side it already reached.
        assertEquals(max, TurretSubsystem.reachableYawDegrees(-178, 120), 1e-9);
        assertEquals(-max, TurretSubsystem.reachableYawDegrees(178, -120), 1e-9);
    }

    /** Servo settles {@code shortDeg} short of its command; first-order 30 ms response. Returns {not-ready ticks, final correction}. */
    private static double[] trackDrift(TurretControl c, double driftDegPerSecond, double shortDeg) {
        double t = 0, dt = .02, measured = 0, velocity = 0, target = 10;
        int notReady = 0;
        for (int i = 0; i < 400; i++) { // 2 s stationary learning, then 6 s of steady drift
            t += dt;
            if (i >= 100) target += driftDegPerSecond * dt;
            double command = c.updateYawCommand(t, target, measured, velocity, true);
            double next = measured + ((command - shortDeg) - measured) * -Math.expm1(-dt / .03);
            velocity = (next - measured) / dt; measured = next;
            if (i >= 100 && !ShotReadiness.angleReady(target - measured)) notReady++;
        }
        return new double[] { notReady, c.getEncoderCorrectionDegrees(), target };
    }

    @Test public void learnedCorrectionIsKeptWhileTrackingAndResetOnlyByATargetJump() {
        for (double drift : new double[] { 0, 2, 5, -5, 20 }) { // 20 deg/s ends at 130 deg, inside the travel limits
            TurretControl c = new TurretControl(); c.setEncoderCorrectionEnabled(true);
            double[] result = trackDrift(c, drift, 4);
            // Previously the offset was dropped every 3 deg of target motion: 17% (2 deg/s) to 90% (5 deg/s) not ready.
            assertEquals("drift " + drift, 0, result[0], 0);
            assertEquals("drift " + drift, 4, result[1], .2);
            // A jump larger than one loop of rated travel is a new episode: start again from the servo position.
            c.updateYawCommand(8.02, result[2] + TurretControl.ENCODER_CORRECTION_RESET_STEP_DEG + 1, result[2], 0, true);
            assertEquals(0, c.getEncoderCorrectionDegrees(), 0);
            assertFalse(c.hasAppliedEncoderCorrection());
        }
        // Losing home also discards it.
        TurretControl c = new TurretControl(); c.setEncoderCorrectionEnabled(true);
        double[] result = trackDrift(c, 2, 4);
        c.updateYawCommand(8.02, result[2], result[2], 0, false);
        assertEquals(0, c.getEncoderCorrectionDegrees(), 0);
    }
}
