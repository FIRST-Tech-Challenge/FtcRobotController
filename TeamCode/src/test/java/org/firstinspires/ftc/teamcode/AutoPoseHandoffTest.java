package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class AutoPoseHandoffTest {
    private static final TurretSubsystem.Alliance RED = TurretSubsystem.Alliance.RED, BLUE = TurretSubsystem.Alliance.BLUE;

    private static MotionSample autoPose(double time, boolean ready, boolean fieldReferenced) {
        return new MotionSample(time, .8, -.6, 1.0, 0, 0, 0, 0, .02, ready, fieldReferenced, "auto");
    }
    @Before public void clearSessionState() { AutoPoseHandoff.clear(); }

    @Test public void sameAllianceRecentPoseIsAppliedOnceAfterThePinpointSettles() {
        AutoPoseHandoff.save(RED, autoPose(10, true, true));
        AutoPoseHandoff.Pending pending = AutoPoseHandoff.take(RED, 15);
        assertTrue(pending.status(), pending.pending());
        assertNull(pending.poll(15.0, false));  // Still CALIBRATING after the TeleOp IMU reset.
        assertNull(pending.poll(15.25, true));  // READY just reported (binary-exact times below).
        assertNull(pending.poll(15.25 + AutoPoseHandoff.PINPOINT_SETTLE_SECONDS - .01, true));
        Pose2D applied = pending.poll(15.25 + AutoPoseHandoff.PINPOINT_SETTLE_SECONDS, true);
        assertNotNull(applied);
        assertEquals(.8, applied.getX(DistanceUnit.METER), 1e-12);
        assertEquals(-.6, applied.getY(DistanceUnit.METER), 1e-12);
        assertEquals(1.0, applied.getHeading(AngleUnit.RADIANS), 1e-12);
        assertTrue(pending.status(), pending.status().startsWith("APPLIED"));
        assertFalse(pending.pending());
        assertNull(pending.poll(20, true)); // Exactly once.
        pending.cancel("late cancel");
        assertTrue(pending.status().startsWith("APPLIED"));
        // Consumed: a second TeleOp INIT in the same session gets nothing.
        assertFalse(AutoPoseHandoff.take(RED, 16).pending());
    }

    @Test public void aCalibrationAfterReadyRestartsTheSettleInterval() {
        AutoPoseHandoff.save(BLUE, autoPose(0, true, true));
        AutoPoseHandoff.Pending pending = AutoPoseHandoff.take(BLUE, 1);
        assertNull(pending.poll(1.0, true));   // Stale READY read before the reset took effect.
        assertNull(pending.poll(1.25, false)); // CALIBRATING.
        assertNull(pending.poll(1.5, true));   // Would have applied here without the restart.
        assertNull(pending.poll(1.5 + AutoPoseHandoff.PINPOINT_SETTLE_SECONDS - .01, true));
        assertNotNull(pending.poll(1.5 + AutoPoseHandoff.PINPOINT_SETTLE_SECONDS, true));
    }

    @Test public void otherAllianceOldOrFuturePosesAreIgnoredAndStillConsumed() {
        AutoPoseHandoff.save(BLUE, autoPose(10, true, true));
        AutoPoseHandoff.Pending wrongAlliance = AutoPoseHandoff.take(RED, 11);
        assertFalse(wrongAlliance.pending());
        assertTrue(wrongAlliance.status(), wrongAlliance.status().contains("BLUE"));
        assertFalse(AutoPoseHandoff.take(BLUE, 11).pending()); // Consumed by the mismatched TeleOp.

        AutoPoseHandoff.save(RED, autoPose(10, true, true));
        assertTrue(AutoPoseHandoff.take(RED, 10 + AutoPoseHandoff.MAX_AGE_SECONDS).pending());
        AutoPoseHandoff.save(RED, autoPose(10, true, true));
        AutoPoseHandoff.Pending old = AutoPoseHandoff.take(RED, 10 + AutoPoseHandoff.MAX_AGE_SECONDS + .5);
        assertFalse(old.pending());
        assertTrue(old.status(), old.status().contains("old"));
        AutoPoseHandoff.save(RED, autoPose(10, true, true));
        assertFalse(AutoPoseHandoff.take(RED, 9).pending()); // Clock went backwards: not trusted.
        assertTrue(AutoPoseHandoff.take(RED, 9).status().startsWith("None"));
    }

    @Test public void onlyReadyFieldReferencedAutonomousPosesAreSaved() {
        AutoPoseHandoff.save(RED, autoPose(10, true, false)); // Cleared-area local reference.
        AutoPoseHandoff.save(RED, autoPose(10, false, true)); // Pinpoint not READY.
        AutoPoseHandoff.save(null, autoPose(10, true, true));
        AutoPoseHandoff.save(RED, null);
        AutoPoseHandoff.save(RED, new MotionSample(10, Double.NaN, 0, 0, 0, 0, 0, 0, .02, true, true, "auto"));
        assertFalse(AutoPoseHandoff.take(RED, 11).pending());
        // The latest valid sample wins; a fault/new autonomous INIT clears it.
        AutoPoseHandoff.save(RED, autoPose(10, true, true));
        AutoPoseHandoff.save(RED, new MotionSample(12, .9, -.5, .5, 0, 0, 0, 0, .02, true, true, "auto"));
        AutoPoseHandoff.Pending latest = AutoPoseHandoff.take(RED, 13);
        latest.poll(13, true);
        assertEquals(.9, latest.poll(14, true).getX(DistanceUnit.METER), 1e-12);
        AutoPoseHandoff.save(RED, autoPose(10, true, true));
        AutoPoseHandoff.clear();
        assertFalse(AutoPoseHandoff.take(RED, 11).pending());
    }

    @Test public void cornerRehomeOrEarlyStartDiscardsAPendingPose() {
        AutoPoseHandoff.save(RED, autoPose(10, true, true));
        AutoPoseHandoff.Pending pending = AutoPoseHandoff.take(RED, 11);
        pending.cancel("Replaced by GP2 D-pad Left corner rehome");
        assertFalse(pending.pending());
        assertEquals("Replaced by GP2 D-pad Left corner rehome", pending.status());
        assertNull(pending.poll(20, true));
    }
}
