package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import org.junit.Test;
import java.util.ArrayList;
import java.util.List;
import static org.junit.Assert.*;

/** Autonomous INIT relocalization: landmark choice, retry/fallback policy and the shot coordinator's look. */
public class AutoStartRelocalizationTest {
    private static final ShooterMount MOUNT = ShootingConfiguration.MOUNT;
    private static final double M = 1 / PedroConfiguration.INCHES_PER_METER;

    private static MotionSample field(double time, double x, double y, double heading) {
        return new MotionSample(time, x, y, heading, 0, 0, 0, 0, .02, true, true, "Synthetic field start");
    }

    @Test public void landmarkIsTheCellRaisedAtTheStartOnTheRobotsHalfForAllFourStarts() {
        // Authored red starts and their blue point mirrors, inches (doc/MatchAutonomousPlan.md).
        double[][] starts = {{-48, -63}, {-10, 63}, {48, 63}, {10, -63}};
        String[] expected = {"RED AUDIENCE", "BLUE FAR", "BLUE FAR", "RED AUDIENCE"};
        for (int i = 0; i < starts.length; i++) {
            GoalTarget landmark = AutoStartRelocalization.landmarkFor(starts[i][1] * M);
            assertEquals(expected[i], landmark.alliance + " " + landmark.side);
        }
        assertEquals(new java.util.TreeSet<>(java.util.Arrays.asList(34, 35, 36, 37)),
                new java.util.TreeSet<>(AutoStartRelocalization.landmarkFor(-1).tags.keySet()));
        assertEquals(new java.util.TreeSet<>(java.util.Arrays.asList(42, 43, 44, 45)),
                new java.util.TreeSet<>(AutoStartRelocalization.landmarkFor(1).tags.keySet()));
    }

    @Test public void waitsThenRequestsRetriesFlagsPlacementAndAppliesARefinementOnce() {
        AutoStartRelocalization r = new AutoStartRelocalization();
        assertEquals(AutoStartRelocalization.Decision.NONE, r.update(false, true, false, "Not requested"));
        assertTrue(r.status(), r.status().contains("turret homing"));
        assertEquals(AutoStartRelocalization.Decision.NONE, r.update(true, false, false, "Not requested"));
        assertTrue(r.status(), r.status().contains("Pinpoint start pose"));
        assertEquals(AutoStartRelocalization.State.WAITING, r.state());

        assertEquals(AutoStartRelocalization.Decision.REQUEST_REFINEMENT, r.update(true, true, false, "Not requested"));
        assertEquals(AutoStartRelocalization.Decision.NONE, r.update(true, true, true, "COLLECTING LIMELIGHT 2/5"));
        assertEquals(AutoStartRelocalization.State.REFINING, r.state());
        assertEquals(AutoStartRelocalization.Decision.NONE,
                r.update(true, true, false, "CORNER ONLY: Limelight refinement timed out"));
        assertEquals(AutoStartRelocalization.State.START_POSE_ONLY, r.state());
        assertFalse(r.checkPlacement());
        assertTrue(r.status(), r.status().startsWith("START POSE ONLY (attempt 1)"));

        // Retried while INIT continues; tags that disagree beyond the bound flag the placement.
        assertEquals(AutoStartRelocalization.Decision.REQUEST_REFINEMENT, r.update(true, true, false, "x"));
        assertEquals(2, r.attempts());
        r.update(true, true, true, "WAITING: Tag 42: correction exceeds corner bound");
        r.update(true, true, false, "CORNER ONLY: Limelight refinement timed out");
        assertTrue(r.checkPlacement());

        assertEquals(AutoStartRelocalization.Decision.REQUEST_REFINEMENT, r.update(true, true, false, "x"));
        r.update(true, true, true, "COLLECTING LIMELIGHT 4/5");
        assertEquals(AutoStartRelocalization.Decision.APPLY_REFINED_POSE,
                r.update(true, true, false, "VISION REFINED: 5 fresh frames + corner reference"));
        assertEquals(AutoStartRelocalization.State.REFINED, r.state());
        assertFalse(r.checkPlacement());
        r.applied(1.234);
        assertTrue(r.status(), r.status().contains("moved 1.2 in"));
        // Applied once: later INIT loops (including the reference change after the write) do nothing.
        assertEquals(AutoStartRelocalization.Decision.NONE, r.update(true, false, false, "Not requested"));
        assertEquals(AutoStartRelocalization.Decision.NONE, r.update(true, true, false, "Not requested"));
    }

    @Test public void refinedPoseIsWrittenToPedroInInchesWithTheSameAxes() {
        Pose pose = BioBuzzRobot.pedroPose(field(1, -.254, 1.6002, .25));
        assertEquals(-10, pose.x(), 1e-9);
        assertEquals(63, pose.y(), 1e-9);
        assertEquals(.25, pose.heading(), 1e-12);
    }

    private static double facing(GoalTarget target, MotionSample robot) {
        return RobotMath.wrap(Math.atan2(target.y - robot.y, target.x - robot.x) - robot.heading - MOUNT.zeroYaw);
    }
    /** Five stationary frames of the landmark's tags seen from {@code actual} while odometry reports {@code seeded}. */
    private static FieldRehomeRefinement refineAtRedFarStart(double dx, double dy, PoseHistory history) {
        MotionSample seeded = field(1, -10 * M, 63 * M, 0);
        MotionSample actual = field(1, seeded.x + dx, seeded.y + dy, 0);
        GoalTarget landmark = AutoStartRelocalization.landmarkFor(seeded.y);
        double yaw = facing(landmark, seeded);
        FieldRehomeRefinement refinement = new FieldRehomeRefinement();
        refinement.request(.99);
        for (int i = 0; i < 10; i++) history.add(field(.8 + i * .02, seeded.x, seeded.y, 0).withReference(8), yaw);
        for (int i = 0; i < FieldRehomeRefinement.REQUIRED_FRAMES; i++) {
            double time = 1 + i * .02;
            history.add(field(time, seeded.x, seeded.y, 0).withReference(8), yaw);
            List<TagObservation> tags = new ArrayList<>();
            for (int id : landmark.tags.keySet()) tags.add(TagFixtures.seen(landmark, id, actual, yaw, MOUNT));
            refinement.update(time, i, time, tags, history, MOUNT);
        }
        return refinement;
    }

    @Test public void redFarStartRefinesFromTheRaisedBlueFarTagsAndFlagsAFarOffPlacement() {
        PoseHistory history = new PoseHistory();
        FieldRehomeRefinement refined = refineAtRedFarStart(.03, -.02, history);
        assertFalse(refined.pending());
        assertTrue(refined.status(), refined.status().startsWith(AutoStartRelocalization.REFINED_PREFIX));
        MotionSample result = history.current(1.08);
        assertTrue(result.x > -10 * M && result.y < 63 * M); // Moved toward the camera's pose.
        assertEquals(0, RobotMath.wrap(result.heading), 1e-9); // Translation only.

        PoseHistory far = new PoseHistory();
        FieldRehomeRefinement misplaced = refineAtRedFarStart(.30, 0, far);
        assertTrue(misplaced.pending());
        assertTrue(misplaced.status(), misplaced.status().contains(AutoStartRelocalization.OUT_OF_BOUND_REASON));
        assertEquals(-10 * M, far.current(1.08).x, 1e-12); // Start pose kept.
    }

    private static TurretSubsystem homedTurret(double[] clock) {
        org.horizon36596.simloop.fakehardware.FakeHardwareMap map = new org.horizon36596.simloop.fakehardware.FakeHardwareMap();
        map.register("turret_motor", new org.horizon36596.simloop.fakehardware.FakeMotor(2800, 8, 28));
        map.register("turret_encoder", new org.horizon36596.simloop.fakehardware.FakeMotor(2800, 8, 28));
        map.register("turret_servo_1", new org.horizon36596.simloop.fakehardware.FakeServo());
        map.register("turret_servo_2", new org.horizon36596.simloop.fakehardware.FakeServo());
        map.register("turret_pitch", new org.horizon36596.simloop.fakehardware.FakeServo());
        map.register("Control Hub", new org.horizon36596.simloop.fakehardware.FakeVoltageSensor(12));
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> clock[0]);
        for (int i = 0; i < 150 && !turret.isHomed(); i++) { clock[0] += .02; turret.update(clock[0]); turret.applyYawCommand(clock[0]); }
        assertTrue(turret.getHomingStatus(), turret.isHomed());
        return turret;
    }

    @Test public void relocalizingAutonomousLooksAtTheChosenLandmarkWhileOdometryOnlyIgnoresRequests() {
        double[] clock = {0};
        TurretSubsystem turret = homedTurret(clock);
        GoalTarget redAudience = TeleOpFieldReference.target(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.AUDIENCE);
        ShotCoordinator odometryOnly = new ShotCoordinator(turret, redAudience, ShotCoordinator.AimMode.AUTONOMOUS_ODOMETRY);
        odometryOnly.requestFieldRehomeRefinement(clock[0], AutoStartRelocalization.landmarkFor(1));
        assertFalse(odometryOnly.fieldRehomePending());

        ShotCoordinator shots = new ShotCoordinator(turret, redAudience, ShotCoordinator.AimMode.AUTONOMOUS_RELOCALIZING);
        MotionSample start = field(clock[0], -10 * M, 63 * M, 0);
        GoalTarget landmark = AutoStartRelocalization.landmarkFor(start.y);
        shots.requestFieldRehomeRefinement(clock[0], landmark);
        assertTrue(shots.fieldRehomePending());
        for (int i = 0; i < 5; i++) {
            clock[0] += .02; turret.update(clock[0]);
            shots.updatePose(clock[0], field(clock[0], start.x, start.y, 0));
        }
        assertTrue(shots.fieldRehomeStatus(), shots.fieldRehomePending());
        double expected = Math.toDegrees(TeleOpFieldReference.yawRadians(start, landmark, MOUNT));
        double redFar = Math.toDegrees(TeleOpFieldReference.yawRadians(start,
                TeleOpFieldReference.target(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.FAR), MOUNT));
        assertEquals(expected, turret.getTargetYawDegrees(), .5);
        assertTrue("the override must differ from the nearest red cell", Math.abs(expected - redFar) > 5);
        shots.cancelFieldRehomeRefinement("START before the refinement finished");
        assertFalse(shots.fieldRehomePending());
    }
}
