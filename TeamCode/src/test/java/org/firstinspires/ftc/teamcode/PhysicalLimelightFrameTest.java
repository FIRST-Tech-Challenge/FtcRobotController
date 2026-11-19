package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import static org.junit.Assert.*;

/**
 * Raw Limelight readings copied from Driver Station photos. SDK roll/pitch/yaw are in degrees and are the
 * raw SDK values (the telemetry shows roll after the +180 deg Z-into-print conversion).
 * The camera is mounted upside down; raw t6t_cs poses are in that inverted sensor frame.
 */
public class PhysicalLimelightFrameTest {
    /** 2026-11-05, Test: Limelight Goal State 3D, Red / Far, raised tags 30-33, robot stationary, 960x720. */
    private static final double[][] RED_FAR_2026_11_05 = {
            // id, sensor right, sensor down, forward (m), roll, pitch, yaw (deg), area (%)
            {30, .186, -.016, 1.117,   5.2,  0.1, -179.6, .007},
            {31, .091, -.017, 1.119,  11.5,  0.8, -179.8, .006},
            {32, -.048, -.017, 1.120, -11.6,  7.4, -179.9, .007},
            {33, -.142, -.017, 1.119,   9.2, -4.0, -179.4, .006},
    };
    /**
     * 2026-11-13, Test: Limelight Goal State 3D at Red audience, 1280x960, robot stationary. Red audience
     * 34-36 and blue audience 38-39 visible. Field-measured: tag centers 47.5-48 in (1.21 m), underside
     * 29.9 deg from horizontal; Limelight face 34.7 deg from horizontal (lens axis 55.3 deg up).
     */
    private static final double[][] AUDIENCE_2026_11_13 = {
            {34, .273, -.154, 1.213,   2.2,   2.6, -179.8, .006},
            {35, .178, -.155, 1.215, -21.3, -18.1, -176.4, .006},
            {36, .040, -.156, 1.207,  10.8,  -0.8, -179.8, .005},
            {38, -.373, -.163, 1.229,  5.3,   0.5, -179.9, .005},
            {39, -.458, -.159, 1.200,  7.6,  -2.8, -179.4, .006},
    };
    private static TagObservation converted(double[] r) {
        return LimelightObservation.robotTag((int) r[0], r[1], r[2], r[3],
                Math.toRadians(r[4]), Math.toRadians(r[5]), Math.toRadians(r[6]), r[7]);
    }
    /** The previous conversion: Z-into-print only, inverted sensor frame left in place. */
    private static TagObservation normalOnly(double[] r) {
        return new TagObservation((int) r[0], r[1], r[2], r[3],
                LimelightObservation.robotTagRoll(Math.toRadians(r[4])), Math.toRadians(r[5]), Math.toRadians(r[6]), r[7]);
    }
    private static TagObservation unconverted(double[] r) {
        return new TagObservation((int) r[0], r[1], r[2], r[3],
                Math.toRadians(r[4]), Math.toRadians(r[5]), Math.toRadians(r[6]), r[7]);
    }
    private static double height(TagObservation tag) {
        return ShootingConfiguration.MOUNT.cameraHeight + tag.levelPosition(ShootingConfiguration.MOUNT)[2];
    }

    @Test public void physicalFrameFacesTheCameraOnlyAfterTheConventionConversion() {
        assertTrue(LimelightObservation.LIMELIGHT_TAG_Z_INTO_PRINT);
        assertTrue(LimelightObservation.LIMELIGHT_MOUNTED_INVERTED);
        for (double[] r : RED_FAR_2026_11_05) {
            assertTrue(unconverted(r).facingCosine() < -.9);
            TagObservation tag = converted(r);
            assertTrue(tag.id + " facing " + tag.facingCosine(), tag.facingCosine() > .9);
            assertNull(tag.id + ": " + tag.rejectionReason(), tag.rejectionReason());
        }
    }

    @Test public void redFarTagOrderAndClusterCenterMatchThePhysicalFrame() {
        // Raw sensor-right decreases from tag 30 to 33, so in the upright camera tag 30 is leftmost.
        for (int i = 1; i < RED_FAR_2026_11_05.length; i++) {
            assertTrue(RED_FAR_2026_11_05[i - 1][1] > RED_FAR_2026_11_05[i][1]);
            assertTrue(converted(RED_FAR_2026_11_05[i - 1]).x < converted(RED_FAR_2026_11_05[i]).x);
        }
        // Robot below the far cell looks toward -Y, so camera-left is field +X: tag 30 has the highest X.
        GoalTarget.Tag[] map = HiveTagGeometry.nominalRaisedTags(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.FAR);
        for (int i = 1; i < map.length; i++) assertTrue(map[i - 1].id + " vs " + map[i].id, map[i - 1].x > map[i].x);
        // Each tag's own +X axis and sticker offset point back to one shared cluster center.
        double[] first = null;
        for (double[] r : RED_FAR_2026_11_05) {
            TagObservation tag = converted(r);
            double[] offset = tag.rotate(-OpeningGeometry.tagOffsetMeters(tag.id), 0, 0);
            double[] center = {tag.x + offset[0], tag.y + offset[1], tag.z + offset[2]};
            if (first == null) first = center;
            assertEquals(tag.id + " right", first[0], center[0], .03);
            assertEquals(tag.id + " forward", first[2], center[2], .03);
            // Upright sticker: +X edge camera-right.
            assertTrue(tag.id + " edge", tag.rotate(1, 0, 0)[0] > .9);
        }
    }

    @Test public void invertedMountConversionRecoversTheMeasuredAudienceTagGeometry() {
        ShooterMount mount = ShootingConfiguration.MOUNT;
        double nominal = ShootingConfiguration.HIGH_GOAL_TAG_CENTER_HEIGHT_METERS;
        for (double[] r : AUDIENCE_2026_11_13) {
            // Leaving the inverted sensor frame in place reads every tag about 0.15 m too high.
            TagObservation old = normalOnly(r);
            assertTrue(old.id + " old height " + height(old), height(old) - nominal > .1);
            assertNotNull(TeleOpFieldReference.raisedTagRejectionReason(old, mount));
            TagObservation tag = converted(r);
            assertEquals(tag.id + " facing unchanged", old.facingCosine(), tag.facingCosine(), 1e-9);
            // Field tape 1.21 m; nominal 1.268 m. All within the acceptance band after conversion.
            assertEquals(tag.id + " height", 1.24, height(tag), .02);
            String rejection = TeleOpFieldReference.raisedTagRejectionReason(tag, mount);
            if (tag.id == 35) {
                // Single-tag orientation outlier (raw pitch -18 deg): rejected per tag, not fused.
                assertNotNull(rejection);
                assertTrue(rejection, rejection.contains("plane"));
            } else {
                assertNull(tag.id + ": " + rejection, rejection);
                assertEquals(tag.id + " plane", HiveTagGeometry.RAISED_TAG_PLANE_DEG, tag.planeDegrees(mount), 6.5);
            }
        }
    }

    @Test public void audienceFrameOrderAndSpacingMatchTheFlippedMap() {
        // Looking +Y from the audience side, camera-right is field +X. Converted right offsets must reproduce
        // the map's X spacing across the red and blue audience cells (both cells are at the same Y).
        TagObservation previous = null;
        for (double[] r : AUDIENCE_2026_11_13) {
            TagObservation tag = converted(r);
            if (previous != null) {
                double measured = tag.x - previous.x;
                double mapped = TeleOpFieldReference.targetForTag(tag.id).tags.get(tag.id).x
                        - TeleOpFieldReference.targetForTag(previous.id).tags.get(previous.id).x;
                assertTrue(previous.id + "->" + tag.id + " left to right", measured > 0);
                assertEquals(previous.id + "->" + tag.id + " spacing", mapped, measured, .015);
            }
            previous = tag;
        }
    }

    @Test public void audienceFrameAcquiresAFieldPoseLookingTowardTheHive() {
        ShooterMount mount = ShootingConfiguration.MOUNT;
        List<TagObservation> tags = new ArrayList<>();
        for (double[] r : AUDIENCE_2026_11_13) {
            TagObservation tag = converted(r);
            if (TeleOpFieldReference.raisedTagRejectionReason(tag, mount) == null) tags.add(tag);
        }
        assertEquals(4, tags.size());
        List<GoalTarget> landmarks = new ArrayList<>();
        for (TagObservation tag : tags) landmarks.add(TeleOpFieldReference.targetForTag(tag.id));
        PoseHistory history = new PoseHistory();
        boolean accepted = false;
        for (double t = .80; t < 1.0 + 1e-9; t += .02) {
            history.add(new MotionSample(t, 0, 0, 0, 0, 0, 0, 0, 0, 1, true, false, "Local Pinpoint").withReference(3), 0);
        }
        for (int i = 0; i < PoseHistory.ACQUISITION_FRAMES; i++) {
            double time = 1.02 + i * .02;
            history.add(new MotionSample(time, 0, 0, 0, 0, 0, 0, 0, 0, 1, true, false, "Local Pinpoint").withReference(3), 0);
            accepted = history.acquireFieldReference3d(time, i + 1, time - .02, tags, landmarks, mount);
        }
        assertTrue(history.reason(), accepted);
        MotionSample pose = history.current(1.06);
        assertTrue(pose.fieldReferenced);
        double cameraYaw = RobotMath.wrap(pose.heading + mount.zeroYaw + mount.cameraYaw);
        assertEquals("camera looks +Y toward the audience cells", Math.PI / 2, cameraYaw, Math.toRadians(5));
        RobotMath.Vec camera = mount.cameraPosition(pose, 0);
        // About 0.82 m in front of the audience tag row, between the red and blue hives.
        assertEquals(-HiveTagGeometry.RAISED_TAG_ROW_Y_METERS - .83, camera.y, .05);
        assertTrue(camera.x > -.35 && camera.x < 0);
    }

    @Test public void reportDownstreamGeometryForThePhysicalFrame() {
        // Diagnostic only: prints what the next checks see.
        ShooterMount mount = ShootingConfiguration.MOUNT;
        List<TagObservation> farTags = new ArrayList<>();
        for (double[][] frame : Arrays.asList(RED_FAR_2026_11_05, AUDIENCE_2026_11_13)) {
            for (double[] r : frame) {
                TagObservation tag = converted(r);
                if (frame == RED_FAR_2026_11_05) farTags.add(tag);
                System.out.printf(java.util.Locale.US, "PHYSICAL tag %d height %.3f m plane %.1f deg | raised check: %s%n",
                        tag.id, height(tag), tag.planeDegrees(mount), TeleOpFieldReference.raisedTagRejectionReason(tag, mount));
            }
        }
        GoalStateObserver observer = new GoalStateObserver(mount);
        observer.select(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.FAR);
        observer.update(1.0, 1, .95, farTags, true);
        System.out.println("PHYSICAL goal observer: " + observer.reason());
    }
}
