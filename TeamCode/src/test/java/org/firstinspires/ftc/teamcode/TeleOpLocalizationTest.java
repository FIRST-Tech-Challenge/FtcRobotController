package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;

public class TeleOpLocalizationTest {
    private static final ShooterMount MOUNT = ShootingConfiguration.MOUNT;
    private static MotionSample local(double time, double x, double y, double heading, double vx, double vy) {
        return new MotionSample(time,x,y,heading,vx,vy,.2,.3,.4,1,true,false,"Local Pinpoint").withReference(7);
    }
    private static MotionSample field(double time, double x, double y, double heading) {
        return new MotionSample(time,x,y,heading,0,0,0,0,.02,true,true,"Synthetic field ground truth");
    }
    /** Project a known field tag (position, edge and outward normal) into the rotating, pitched camera. */
    private static TagObservation observation(GoalTarget target, int id, MotionSample actual, double turretYaw) {
        return TagFixtures.seen(target,id,actual,turretYaw,MOUNT);
    }
    /** Fusion needs settled capture-time history; seed 0.1 s of stationary samples before the test. */
    private static void settled(PoseHistory history, double until, double turretYaw) {
        for (double t = until-.12; t < until-1e-9; t += .02) history.add(local(t,0,0,0,0,0),turretYaw);
    }
    private static GoalTarget audience() { return TeleOpFieldReference.targetForTag(38); }
    @Test public void rejectedMetricPosesReportTheSameCauseWithoutCreatingAFieldReference() {
        TagObservation backwards = new TagObservation(38,.186,-.015,1.113,0,0,0,.007);
        assertTrue(TeleOpFieldReference.raisedTagRejectionReason(backwards,MOUNT).contains("Facing check failed"));
        PoseHistory history = new PoseHistory();
        for (int i = 0; i < 20; i++) history.add(local(1+i*.02,0,0,0,0,0),0);
        assertFalse(history.acquireFieldReference3d(1.38,1,1.2,backwards,audience(),MOUNT));
        assertTrue(history.reason(),history.reason().contains("Tag 38: Facing check failed"));
        assertFalse(history.current(1.38).fieldReferenced);
        assertEquals(Double.POSITIVE_INFINITY,history.observationAge(1.38),0);
        assertFalse(history.observe3d(1.38,2,1.2,backwards,audience(),MOUNT));
        assertTrue(history.reason(),history.reason().contains("Tag 38: Facing check failed"));

        PoseHistory corner = new PoseHistory();
        for (int i = 0; i < 20; i++) corner.add(field(1+i*.02,.8,-1.3,2.8),0);
        FieldRehomeRefinement refinement = new FieldRehomeRefinement(); refinement.request(1.05);
        assertFalse(refinement.update(1.38,1,1.2,java.util.Collections.singletonList(backwards),corner,MOUNT));
        assertTrue(refinement.status(),refinement.status().contains("Tag 38: Facing check failed"));
        assertEquals(.8,corner.current(1.38).x,0);
        assertEquals(-1.3,corner.current(1.38).y,0);
    }
    private static ShotProfile calibrated(GoalTarget target) {
        return new ShotProfile(target.geometryId,true,new ShotProfile.Row(.2,1,3000),new ShotProfile.Row(5,10,3400));
    }
    private static double turretFacing(GoalTarget target, MotionSample actual) {
        return RobotMath.wrap(Math.atan2(target.y-actual.y,target.x-actual.x)-actual.heading-MOUNT.zeroYaw);
    }
    @Test public void cornerRefinementFusesFiveMultiTagFramesWithPriorAndPreservesHeading() {
        for (TurretSubsystem.Alliance alliance : TurretSubsystem.Alliance.values()) {
            double sign = alliance == TurretSubsystem.Alliance.BLUE ? 1 : -1;
            double x = sign*1.566319, y = -sign*1.67045, heading = sign > 0 ? Math.PI : 0;
            GoalTarget target = TeleOpFieldReference.target(alliance, sign > 0
                    ? TurretSubsystem.HiveSide.AUDIENCE : TurretSubsystem.HiveSide.FAR);
            MotionSample actual = field(1,x-sign*.025,y+sign*.02,heading);
            double yaw = turretFacing(target,actual);
            PoseHistory history = new PoseHistory();
            FieldRehomeRefinement refinement = new FieldRehomeRefinement();
            refinement.request(.99);
            for (int i = 0; i < 10; i++) history.add(field(.8+i*.02,x,y,heading).withReference(8),yaw);
            for (int i = 0; i < FieldRehomeRefinement.REQUIRED_FRAMES; i++) {
                double time = 1+i*.02;
                history.add(field(time,x,y,heading).withReference(8),yaw);
                java.util.List<TagObservation> tags = new java.util.ArrayList<>();
                for (int id : target.tags.keySet()) tags.add(observation(target,id,actual,yaw));
                boolean accepted = refinement.update(time,i,time,tags,history,MOUNT);
                assertEquals(i == FieldRehomeRefinement.REQUIRED_FRAMES-1,accepted);
                if (!accepted) assertEquals(x,history.current(time).x,0);
            }
            MotionSample result = history.current(1.08);
            assertFalse(refinement.pending()); assertTrue(refinement.status().contains("VISION REFINED"));
            assertTrue(Math.hypot(result.x-actual.x,result.y-actual.y) < Math.hypot(x-actual.x,y-actual.y));
            assertTrue(sign*(result.x-x) < 0); assertTrue(sign*(result.y-y) > 0);
            assertEquals(0,RobotMath.wrap(result.heading-heading),1e-9);
            assertEquals(0,result.vx,0); assertEquals(0,result.acceleration,0);
            assertTrue(result.uncertainty > 0 && result.uncertainty < .02);
            assertEquals(0,history.observationAge(1.08),1e-9);
        }
    }
    @Test public void cornerRefinementRejectsOldDuplicateUnmappedLoweredAndJumpingFrames() {
        PoseHistory history = new PoseHistory();
        GoalTarget target = audience();
        MotionSample actual = field(1,.8,-1.3,2.8);
        double yaw = turretFacing(target,actual);
        for (int i = 0; i < 20; i++) history.add(field(1+i*.02,.8,-1.3,2.8).withReference(8),yaw);
        FieldRehomeRefinement refinement = new FieldRehomeRefinement(); refinement.request(1.1);
        TagObservation good = observation(target,38,actual,yaw);
        assertFalse(refinement.update(1.38,1,1.08,java.util.Arrays.asList(good),history,MOUNT));
        assertFalse(refinement.update(1.38,2,1.2,java.util.Arrays.asList(new TagObservation(29,good.x,good.y,good.z,good.roll,good.pitch,good.yaw,1)),history,MOUNT));
        TagObservation low = new TagObservation(38,good.x,good.y+.5,good.z,good.roll,good.pitch,good.yaw,1);
        assertFalse(refinement.update(1.38,3,1.2,java.util.Arrays.asList(low),history,MOUNT));
        TagObservation jump = observation(target,38,field(1,1.2,-1.3,2.8),yaw);
        assertFalse(refinement.update(1.38,4,1.2,java.util.Arrays.asList(jump),history,MOUNT));
        assertFalse(refinement.update(1.38,5,1.2,java.util.Arrays.asList(good),history,MOUNT));
        assertTrue(refinement.status().contains("1/5"));
        for (int i = 0; i < 10; i++) assertFalse(refinement.update(1.38,5,1.2,java.util.Arrays.asList(good),history,MOUNT));
        assertTrue(refinement.status().contains("1/5"));
        assertEquals(.8,history.current(1.38).x,0);
        refinement.expire(4.11);
        assertFalse(refinement.pending()); assertTrue(refinement.status().contains("CORNER ONLY"));
        assertEquals(.8,history.current(1.38).x,0);
    }
    @Test public void cornerRefinementCancelsOnMotionAndReferenceChange() {
        PoseHistory history = new PoseHistory(); FieldRehomeRefinement refinement = new FieldRehomeRefinement();
        refinement.request(.99);
        history.add(field(1,.8,-1.3,0).withReference(8),0);
        refinement.update(1,1,1,java.util.Collections.emptyList(),history,MOUNT);
        history.add(field(1.02,.8,-1.3,0).withReference(9),0);
        refinement.update(1.02,2,1.02,java.util.Collections.emptyList(),history,MOUNT);
        assertFalse(refinement.pending()); assertTrue(refinement.status().contains("reference changed"));
        refinement.request(1.03);
        history.add(new MotionSample(1.04,.8,-1.3,0,.2,0,0,0,.02,true,true,"moving").withReference(9),0);
        refinement.update(1.04,3,1.04,java.util.Collections.emptyList(),history,MOUNT);
        assertFalse(refinement.pending()); assertTrue(refinement.status().contains("robot moved"));
    }
    @Test public void cornerRefinementRejectsFramesCapturedWhileTurretTurns() {
        PoseHistory history = new PoseHistory();
        GoalTarget target = audience(); MotionSample actual = field(1,.8,-1.3,2.8);
        double startYaw = turretFacing(target,actual);
        for (int i = 0; i < 20; i++) history.add(field(1+i*.02,.8,-1.3,2.8).withReference(8),startYaw+i*.02);
        FieldRehomeRefinement refinement = new FieldRehomeRefinement(); refinement.request(1.05);
        for (int i = 0; i < 5; i++) {
            double capture = 1.2+i*.02;
            assertFalse(refinement.update(1.38,i,capture,
                    java.util.Arrays.asList(observation(target,38,actual,startYaw+capture-1)),history,MOUNT));
        }
        assertTrue(refinement.pending());
        assertEquals(.8,history.current(1.38).x,0);
        assertFalse(refinement.status().contains("COLLECTING"));
    }
    @Test public void everyMappedTagAcquiresUnknownPositionAndHeadingWithoutCornerAssumption() {
        for (int id = 30; id <= 45; id++) {
            GoalTarget target = TeleOpFieldReference.targetForTag(id);
            PoseHistory history = new PoseHistory();
            double y = target.side == TurretSubsystem.HiveSide.AUDIENCE ? -1.3 : 1.3;
            double heading = id % 2 == 0 ? 2.8 : -2.9;
            settled(history,1,turretFacing(target,field(1,.85,y,heading)));
            for (int i = 0; i < PoseHistory.ACQUISITION_FRAMES; i++) {
                double time = 1+i*.02;
                MotionSample actual = field(time,.85,y,heading);
                double turretYaw = turretFacing(target,actual);
                history.add(local(time,0,0,0,0,0),turretYaw);
                TagObservation tag = observation(target,id,actual,turretYaw);
                assertTrue(tag.usable());
                boolean accepted = history.acquireFieldReference3d(time,i,time,tag,target,MOUNT);
                assertEquals(history.reason(),i == PoseHistory.ACQUISITION_FRAMES-1,accepted);
                assertEquals(accepted,history.current(time).fieldReferenced);
            }
            MotionSample result = history.current(1.04);
            assertEquals(.85,result.x,1e-8); assertEquals(y,result.y,1e-8);
            assertEquals(0,RobotMath.wrap(result.heading-heading),1e-8);
        }
        assertNull(TeleOpFieldReference.targetForTag(29));
        assertNull(TeleOpFieldReference.targetForTag(46));
    }
    @Test public void captureHistoryCompensatesMotionAndRotatesVelocityIntoFieldAxes() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        double rotation = 2.8;
        for (int i = -12; i <= 12; i++) {
            double t = 1+i*.01;
            history.add(local(t,.2*(t-1),-.1*(t-1),.2*(t-1),.2,-.1),.4+.3*(t-1));
        }
        for (int i = 0; i < 3; i++) {
            double capture = 1.04+i*.02;
            Vec position = new Vec(.2*(capture-1),-.1*(capture-1)).rotate(rotation).plus(new Vec(.8,-1.3));
            MotionSample actual = field(capture,position.x,position.y,rotation+.2*(capture-1));
            double yaw = .4+.3*(capture-1);
            boolean accepted = history.acquireFieldReference3d(1.12,i,capture,observation(target,38,actual,yaw),target,MOUNT);
            assertEquals(history.reason(),i == 2,accepted);
        }
        // No camera for the rest of a match: the same transform continues to track measured motion.
        for (int i = 13; i <= 18000; i++) {
            double t = 1+i*.01;
            history.add(local(t,.2*(t-1),-.1*(t-1),.2*(t-1),.2,-.1),.4);
        }
        MotionSample result = history.current(181);
        Vec expected = new Vec(36,-18).rotate(rotation).plus(new Vec(.8,-1.3));
        Vec velocity = new Vec(.2,-.1).rotate(rotation);
        assertTrue(result.fieldReferenced); assertTrue(result.fresh(181,.1));
        assertEquals(expected.x,result.x,1e-8); assertEquals(expected.y,result.y,1e-8);
        assertEquals(velocity.x,result.vx,1e-8); assertEquals(velocity.y,result.vy,1e-8);
        assertEquals(0,RobotMath.wrap(result.heading-rotation-36),1e-8);
        assertEquals(.2,result.omega,0); assertEquals(.3,result.acceleration,0);
        assertEquals(.4,result.angularAcceleration,0);
        assertFalse(result.fresh(182,.1));
        history.add(local(181.02,0,0,0,0,0).withReference(8),0);
        assertFalse(history.current(181.02).fieldReferenced);
    }
    @Test public void explicitCornerRehomeDiscardsPreviousVisionTransformAndCaptureHistory() {
        PoseHistory history = new PoseHistory();
        GoalTarget target = audience();
        MotionSample actual = field(1,.8,-1.3,2.8);
        double yaw = turretFacing(target,actual);
        settled(history,1,yaw);
        for (int i = 0; i < PoseHistory.ACQUISITION_FRAMES; i++) {
            double time = 1+i*.02;
            history.add(local(time,0,0,0,0,0),yaw);
            history.acquireFieldReference3d(time,i,time,observation(target,38,actual,yaw),target,MOUNT);
        }
        assertEquals(.8,history.current(1.04).x,1e-8);
        MotionSample rehome = field(1.06,-1.566319,1.67045,0).withReference(8);
        history.add(rehome,yaw);
        MotionSample result = history.current(1.06);
        assertTrue(result.fieldReferenced);
        assertEquals(rehome.x,result.x,0);
        assertEquals(rehome.y,result.y,0);
        assertEquals(rehome.heading,result.heading,0);
        assertNull(history.at(1.04));
        assertFalse(history.observe3d(1.06,4,1.04,observation(target,38,actual,yaw),target,MOUNT));
    }
    @Test public void duplicateStaleInconsistentAndWrongHeightFramesCannotBootstrap() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        MotionSample actual = field(1,.8,-1.3,2.8);
        double yaw = turretFacing(target,actual);
        TagObservation good = observation(target,38,actual,yaw);
        for (int i = 0; i <= 50; i++) history.add(local(1+i*.01,0,0,0,0,0),yaw);
        assertFalse(history.acquireFieldReference3d(1.1,1,1.05,good,target,MOUNT));
        for (int i = 0; i < 5; i++) assertFalse(history.acquireFieldReference3d(1.1,1,1.05,good,target,MOUNT));
        assertFalse(history.current(1.1).fieldReferenced);
        assertFalse(history.acquireFieldReference3d(1.5,2,1,good,target,MOUNT));
        assertFalse(history.acquireFieldReference3d(1.5,3,1.6,good,target,MOUNT));
        assertFalse(history.acquireFieldReference3d(1.5,4,.5,good,target,MOUNT));
        TagObservation low = new TagObservation(38,good.x,good.y+.5,good.z,good.roll,good.pitch,good.yaw,1);
        assertFalse(history.acquireFieldReference3d(1.5,5,1.45,low,target,MOUNT));
        assertFalse(history.acquireFieldReference3d(1.5,6,1.45,good,target,MOUNT));
        TagObservation shifted = observation(target,38,field(1,.8,-1.6,2.8),yaw);
        assertFalse(history.acquireFieldReference3d(1.5,7,1.46,shifted,target,MOUNT));
        assertFalse(history.acquireFieldReference3d(1.5,8,1.47,good,target,MOUNT));
        assertFalse(history.current(1.5).fieldReferenced);
        ShotPlanner.Solution blocked = new ShotPlanner(MOUNT).solve(1.5,history.current(1.5),target,calibrated(target));
        assertFalse(blocked.aimable); assertTrue(blocked.reason.contains("field pose"));
    }
    @Test public void subsequentTranslationCorrectionsPreserveHeadingAndReferenceAcrossTargetOrTurretChanges() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        MotionSample actual = field(1,.8,-1.3,2.8);
        double yaw = turretFacing(target,actual);
        TagObservation tag = observation(target,38,actual,yaw);
        settled(history,1,yaw);
        for (int i = 0; i < 3; i++) {
            double time = 1+i*.02;
            history.add(local(time,0,0,0,.2,0),yaw);
            history.acquireFieldReference3d(time,i,time,tag,target,MOUNT);
        }
        ShotPlanner.Solution localized = new ShotPlanner(MOUNT).solve(1.04,history.current(1.04),target,calibrated(target));
        assertTrue(localized.reason,localized.valid);
        history.invalidateObservation(false); // Nearest scoring cell changed.
        assertTrue(history.current(1.04).fieldReferenced);
        history.invalidateObservation(true); // Turret rehomed; capture history must be rebuilt.
        for (double t = .94; t < 1.059; t += .02) history.add(local(t,.1,0,0,.2,0),yaw);
        history.add(local(1.06,.1,0,0,.2,0),yaw);
        Vec drifted = new Vec(.1,0).rotate(2.8).plus(new Vec(.8,-1.3));
        assertTrue(history.observe3d(1.06,3,1.06,tag,target,MOUNT));
        MotionSample corrected = history.current(1.06);
        assertEquals(.8+.65*(drifted.x-.8),corrected.x,1e-8);
        assertEquals(-1.3+.65*(drifted.y+1.3),corrected.y,1e-8);
        assertEquals(2.8,corrected.heading,1e-8);
        TagObservation jump = observation(target,38,field(1,.8,-1.3-.8,2.8),yaw);
        assertFalse(history.observe3d(1.06,4,1.06,jump,target,MOUNT));
        assertEquals(corrected.x,history.current(1.06).x,0);
    }

    private static java.util.List<TagObservation> cell(GoalTarget target, MotionSample actual, double turretYaw) {
        java.util.List<TagObservation> tags = new java.util.ArrayList<>();
        for (GoalTarget.Tag known : target.tags.values()) tags.add(observation(target,known.id,actual,turretYaw));
        return tags;
    }
    private static java.util.List<GoalTarget> landmarks(java.util.List<TagObservation> tags) {
        java.util.List<GoalTarget> targets = new java.util.ArrayList<>();
        for (TagObservation tag : tags) targets.add(TeleOpFieldReference.targetForTag(tag.id));
        return targets;
    }
    /** Acquire on the audience cell at (.8, -1.3, 2.8) from a stationary local origin; returns next frame id. */
    private static int acquireAudience(PoseHistory history, double turretYaw) {
        GoalTarget target = audience();
        MotionSample actual = field(1,.8,-1.3,2.8);
        settled(history,1,turretYaw);
        for (int i = 0; i < PoseHistory.ACQUISITION_FRAMES; i++) {
            double time = 1+i*.02;
            history.add(local(time,0,0,0,0,0),turretYaw);
            java.util.List<TagObservation> tags = cell(target,actual,turretYaw);
            assertEquals(history.reason(),i == PoseHistory.ACQUISITION_FRAMES-1,
                    history.acquireFieldReference3d(time,i,time,tags,landmarks(tags),MOUNT));
        }
        return PoseHistory.ACQUISITION_FRAMES;
    }
    @Test public void turretSwingAtCaptureDefersFieldCorrectionWithoutMovingThePose() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        double yaw = turretFacing(target,field(1,.8,-1.3,2.8));
        int frame = acquireAudience(history,yaw);
        // RT starts a fast turret slew (2 rad/s) while the robot stays still.
        for (int i = 1; i <= 10; i++) history.add(local(1.04+i*.02,0,0,0,0,0),yaw+2*i*.02);
        MotionSample before = history.current(1.24);
        double capture = 1.20;
        java.util.List<TagObservation> tags = cell(target,field(capture,.8,-1.3,2.8),yaw+2*.16);
        tags.set(0,new TagObservation(tags.get(0).id,tags.get(0).x+.05,tags.get(0).y,tags.get(0).z,
                tags.get(0).roll,tags.get(0).pitch,tags.get(0).yaw,1)); // Latency-smeared view.
        assertFalse(history.observe3d(1.24,frame,capture,tags,landmarks(tags),MOUNT));
        assertTrue(history.reason(),history.reason().contains("Turret moving"));
        MotionSample after = history.current(1.24);
        assertEquals(before.x,after.x,0); assertEquals(before.y,after.y,0); assertEquals(before.heading,after.heading,0);
    }
    @Test public void laterFramesCorrectHeadingGraduallyAboutTheCapturePosition() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        double yaw = turretFacing(target,field(1,.8,-1.3,2.8));
        int frame = acquireAudience(history,yaw);
        // Local odometry heading drifts 0.05 rad, then holds steady; the true robot never moved.
        for (double t = 1.06; t < 1.199; t += .02) history.add(local(t,0,0,.05,0,0),yaw);
        double previous = Double.POSITIVE_INFINITY;
        for (int k = 0; k < 12; k++) {
            double t = 1.20+k*.02;
            history.add(local(t,0,0,.05,0,0),yaw);
            java.util.List<TagObservation> tags = cell(target,field(t,.8,-1.3,2.8),yaw);
            assertTrue(history.reason(),history.observe3d(t,frame++,t,tags,landmarks(tags),MOUNT));
            MotionSample pose = history.current(t);
            double error = Math.abs(RobotMath.wrap(pose.heading-2.8));
            assertTrue(error < previous); previous = error;
            assertEquals(.8,pose.x,.002); assertEquals(-1.3,pose.y,.002);
        }
        assertTrue(previous < Math.toRadians(.1));
    }
    @Test public void perTagEdgeNoiseAveragesWithinOneFrame() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        MotionSample actual = field(1,.8,-1.3,2.8);
        double yaw = turretFacing(target,actual), noise = Math.toRadians(4);
        settled(history,1,yaw);
        double singleError = 0;
        for (int i = 0; i < PoseHistory.ACQUISITION_FRAMES; i++) {
            double time = 1+i*.02;
            history.add(local(time,0,0,0,0,0),yaw);
            java.util.List<TagObservation> tags = cell(target,actual,yaw);
            for (int j = 0; j < tags.size(); j++) {
                TagObservation t = tags.get(j);
                tags.set(j,new TagObservation(t.id,t.x,t.y,t.z,t.roll,t.pitch,t.yaw+(j % 2 == 0 ? noise : -noise),1));
            }
            PoseHistory.Entry entry = history.at(time);
            singleError = Math.max(singleError,Math.abs(RobotMath.wrap(PoseHistory.tagHeading(tags.get(0),target,
                    new PoseHistory.Entry(entry.pose,entry.turretYaw),MOUNT)-2.8)));
            history.acquireFieldReference3d(time,i,time,tags,landmarks(tags),MOUNT);
        }
        assertTrue(history.reason(),history.current(1.04).fieldReferenced);
        assertTrue(singleError > Math.toRadians(2));
        assertEquals(2.8,history.current(1.04).heading,Math.toRadians(.5));
    }

    /** Odometry has drifted by (dx, dy) local metres and dh radians; the camera sees the true pose. */
    private static int driftFrames(PoseHistory history, int frame, double start, int count, double dx, double dy, double dh,
                                   double yaw, java.util.List<Boolean> results) {
        GoalTarget target = audience();
        for (int k = 0; k < count; k++) {
            double t = start+k*.02;
            history.add(local(t,dx,dy,dh,0,0),yaw);
            java.util.List<TagObservation> tags = cell(target,field(t,.8,-1.3,2.8),yaw);
            results.add(history.checkDrift3d(t,frame++,t,tags,landmarks(tags),MOUNT));
        }
        return frame;
    }
    @Test public void driftWithinDeadbandNeverMovesThePose() {
        PoseHistory history = new PoseHistory();
        double yaw = turretFacing(audience(),field(1,.8,-1.3,2.8));
        int frame = acquireAudience(history,yaw);
        for (double t = 1.06; t < 1.199; t += .02) history.add(local(t,.02,0,0,0,0),yaw);
        Vec odometry = new Vec(.02,0).rotate(2.8).plus(new Vec(.8,-1.3));
        java.util.List<Boolean> results = new java.util.ArrayList<>();
        driftFrames(history,frame,1.20,12,.02,0,0,yaw,results);
        assertFalse(results.contains(false));
        assertTrue(history.reason(),history.reason().contains("Odometry agrees with camera"));
        MotionSample pose = history.current(1.42);
        assertEquals(odometry.x,pose.x,1e-9); assertEquals(odometry.y,pose.y,1e-9); assertEquals(2.8,pose.heading,1e-9);
    }
    @Test public void consistentDriftIsCorrectedOnceAndBlendedWithoutAJump() {
        PoseHistory history = new PoseHistory();
        double yaw = turretFacing(audience(),field(1,.8,-1.3,2.8));
        int frame = acquireAudience(history,yaw);
        // Odometry drifts 10 cm and 2 deg, then holds steady long enough to count as settled.
        for (double t = 1.06; t < 1.199; t += .02) history.add(local(t,.1,0,Math.toRadians(2),0,0),yaw);
        MotionSample drifted = history.current(1.18);
        java.util.List<Boolean> results = new java.util.ArrayList<>();
        frame = driftFrames(history,frame,1.20,PoseHistory.DRIFT_FRAMES-1,.1,0,Math.toRadians(2),yaw,results);
        assertEquals(drifted.x,history.current(1.26).x,1e-9); // Still checking: no change yet.
        frame = driftFrames(history,frame,1.28,1,.1,0,Math.toRadians(2),yaw,results);
        assertFalse(results.contains(false));
        assertTrue(history.reason(),history.reason().contains("Odometry drift corrected"));
        MotionSample previous = history.current(1.28);
        for (double t = 1.30; t < 1.80; t += .02) {
            history.add(local(t,.1,0,Math.toRadians(2),0,0),yaw);
            MotionSample pose = history.current(t);
            assertTrue("step " + Math.hypot(pose.x-previous.x,pose.y-previous.y),
                    Math.hypot(pose.x-previous.x,pose.y-previous.y) < .012);
            assertTrue(Math.abs(RobotMath.wrap(pose.heading-previous.heading)) < Math.toRadians(.25));
            previous = pose;
        }
        assertEquals(.8,previous.x,.005); assertEquals(-1.3,previous.y,.005);
        assertEquals(2.8,previous.heading,Math.toRadians(.2));
    }
    @Test public void largeCameraDisagreementIsReportedAndIgnored() {
        PoseHistory history = new PoseHistory();
        double yaw = turretFacing(audience(),field(1,.8,-1.3,2.8));
        int frame = acquireAudience(history,yaw);
        for (double t = 1.06; t < 1.199; t += .02) history.add(local(t,.7,0,0,0,0),yaw);
        MotionSample before = history.current(1.18);
        java.util.List<Boolean> results = new java.util.ArrayList<>();
        driftFrames(history,frame,1.20,8,.7,0,0,yaw,results);
        assertFalse(results.contains(true));
        assertTrue(history.reason(),history.reason().contains("Camera disagrees with odometry"));
        assertEquals(before.x,history.current(1.34).x,1e-9); assertEquals(before.y,history.current(1.34).y,1e-9);
    }
    @Test public void largeConsistentDisagreementReLocalizesAfterConfirmationWithoutAJump() {
        GoalTarget target = audience(); PoseHistory history = new PoseHistory();
        double yaw = turretFacing(target,field(1,.8,-1.3,2.8));
        int frame = acquireAudience(history,yaw);
        // Pods slipped 70 cm and the heading is 11.5 deg off (a collision, or a handoff from the wrong place).
        double dh = .2;
        for (double t = 1.06; t < 1.199; t += .02) history.add(local(t,.7,0,dh,0,0),yaw);
        MotionSample wrong = history.current(1.18);
        boolean relocalized = false;
        int count = 0;
        double t = 1.20;
        while (!relocalized && count < 60) {
            t = 1.20+count*.02; count++;
            history.add(local(t,.7,0,dh,0,0),yaw);
            java.util.List<TagObservation> tags = cell(target,field(t,.8,-1.3,2.8),yaw);
            relocalized = history.checkDrift3d(t,frame++,t,tags,landmarks(tags),MOUNT);
            if (!relocalized) {
                assertTrue(history.reason(),history.reason().contains("Camera disagrees with odometry") && history.reason().contains("confirming"));
                assertTrue(history.largeCorrectionStatus(t+1e-6).startsWith("CAMERA DISAGREES"));
                assertEquals(wrong.x,history.current(t).x,1e-9); // Still confirming: unchanged.
            }
        }
        assertTrue(history.reason(),relocalized);
        assertTrue(history.reason(),history.reason().contains("re-localized"));
        assertTrue(count >= PoseHistory.RELOCALIZE_FRAMES);
        assertEquals(PoseHistory.RELOCALIZE_MIN_SPAN_SECONDS,t-1.20,.02+1e-9); // Needed (about) the full span, no more.
        double last = t-.02;
        assertTrue(history.largeCorrectionStatus(t+1e-6).startsWith("RE-LOCALIZED"));
        // Blended over 0.4 s like drift: bounded steps, then the true pose.
        MotionSample previous = history.current(t);
        for (t = last+.04; t < last+.7; t += .02) {
            history.add(local(t,.7,0,dh,0,0),yaw);
            MotionSample pose = history.current(t);
            assertTrue("step " + Math.hypot(pose.x-previous.x,pose.y-previous.y), Math.hypot(pose.x-previous.x,pose.y-previous.y) < .06);
            assertTrue(Math.abs(RobotMath.wrap(pose.heading-previous.heading)) < Math.toRadians(.7));
            previous = pose;
        }
        assertEquals(.8,previous.x,.005); assertEquals(-1.3,previous.y,.005);
        assertEquals(2.8,previous.heading,Math.toRadians(.2));
        assertNull(history.largeCorrectionStatus(last+.02+PoseHistory.RELOCALIZED_DISPLAY_SECONDS+.1));
    }
    @Test public void singleTagInconsistentOrInterruptedLargeDisagreementNeverReLocalizes() {
        GoalTarget target = audience();
        double yaw = turretFacing(target,field(1,.8,-1.3,2.8));
        // 1) One tag per frame: never enough to overrule odometry.
        PoseHistory history = new PoseHistory();
        int frame = acquireAudience(history,yaw);
        for (double t = 1.06; t < 1.199; t += .02) history.add(local(t,.7,0,0,0,0),yaw);
        MotionSample wrong = history.current(1.18);
        for (int k = 0; k < 75; k++) {
            double t = 1.20+k*.02;
            history.add(local(t,.7,0,0,0,0),yaw);
            java.util.List<TagObservation> one = new java.util.ArrayList<>(cell(target,field(t,.8,-1.3,2.8),yaw).subList(0,1));
            assertFalse(history.checkDrift3d(t,frame++,t,one,landmarks(one),MOUNT));
        }
        assertTrue(history.reason(),history.reason().contains("2+ tags"));
        assertTrue(history.largeCorrectionStatus(2.69).contains("tags in view"));
        assertEquals(wrong.x,history.current(2.68).x,1e-9); assertEquals(wrong.y,history.current(2.68).y,1e-9);
        // 2) Camera poses that disagree with each other by 10 cm: inconsistent, odometry kept.
        history = new PoseHistory();
        frame = acquireAudience(history,yaw);
        for (double t = 1.06; t < 1.199; t += .02) history.add(local(t,.7,0,0,0,0),yaw);
        wrong = history.current(1.18);
        boolean inconsistent = false;
        for (int k = 0; k < 75; k++) {
            double t = 1.20+k*.02;
            history.add(local(t,.7,0,0,0,0),yaw);
            java.util.List<TagObservation> tags = cell(target,field(t,k % 2 == 0 ? .8 : .9,-1.3,2.8),yaw);
            assertFalse(history.checkDrift3d(t,frame++,t,tags,landmarks(tags),MOUNT));
            inconsistent |= history.reason().contains("inconsistent");
        }
        assertTrue(inconsistent);
        assertEquals(wrong.x,history.current(2.68).x,1e-9);
        // 3) A frame that agrees with odometry restarts the confirmation: two 0.38-s runs never add up to 0.5 s.
        history = new PoseHistory();
        frame = acquireAudience(history,yaw);
        for (double t = 1.06; t < 1.199; t += .02) history.add(local(t,.7,0,0,0,0),yaw);
        java.util.List<Boolean> results = new java.util.ArrayList<>();
        frame = driftFrames(history,frame,1.20,20,.7,0,0,yaw,results);
        double t = 1.60;
        history.add(local(t,.7,0,0,0,0),yaw);
        MotionSample odometry = history.current(t);
        java.util.List<TagObservation> agreeing = cell(target,field(t,odometry.x,odometry.y,odometry.heading),yaw);
        assertTrue(history.reason(),history.checkDrift3d(t,frame++,t,agreeing,landmarks(agreeing),MOUNT));
        driftFrames(history,frame,t+.02,20,.7,0,0,yaw,results);
        assertFalse(results.contains(true));
        assertTrue(history.reason(),history.reason().contains("confirming"));
    }
}
