package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import java.util.*;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class GoalObservationTest {
    private final ShooterMount mount=ShootingConfiguration.MOUNT;
    /** Independently project a raised underside: normal points toward viewer and down, tag X right. */
    private TagObservation tag(int id,double height,double planeDeg,double cameraPitch) {
        double forward=2, up=height-mount.cameraHeight;
        double y=forward*Math.sin(cameraPitch)-up*Math.cos(cameraPitch);
        double z=forward*Math.cos(cameraPitch)+up*Math.sin(cameraPitch);
        // Camera optical +Y down; Rx rotates the +Z normal to down/toward viewer.
        double roll=Math.toRadians(-90-planeDeg)-cameraPitch;
        return new TagObservation(id,OpeningGeometry.tagOffsetMeters(id),y,z,roll,0,0,1);
    }
    private GoalStateObserver observer(int id) {
        GoalStateObserver o=new GoalStateObserver(mount,.8,30);
        o.select(id<38?TurretSubsystem.Alliance.RED:TurretSubsystem.Alliance.BLUE,
                (id<34||id>=42)?TurretSubsystem.HiveSide.FAR:TurretSubsystem.HiveSide.AUDIENCE);
        return o;
    }
    private void qualify(GoalStateObserver o,int id,double start,double height) {
        for(int i=0;i<4;i++) { double t=start+i*.05; o.update(t+.01,t,t,Collections.singletonList(tag(id,height,30,mount.cameraPitch)),true); }
    }
    @Test public void allCellsRequireDistinctStableFramesBeforeRaisedPermission() {
        for(int id:new int[]{30,34,38,42}) {
            GoalStateObserver o=observer(id);
            o.update(1.01,1,1,Collections.singletonList(tag(id,1.25,30,mount.cameraPitch)),true);
            assertFalse(o.raised(1.01));
            o.update(1.12,1,1,Collections.singletonList(tag(id,1.25,30,mount.cameraPitch)),true);
            assertFalse(o.raised(1.12));
            qualify(o,id,1.13,1.25); assertTrue(o.reason(),o.raised(1.29));
        }
    }
    @Test public void loweredOnlyWhenItsIndependentExpectedPoseIsSupplied() {
        GoalStateObserver o=observer(30); qualify(o,30,1,.8);
        assertEquals(o.reason(),GoalStateObserver.State.LOWERED,o.state(1.16));
        o=new GoalStateObserver(mount); o.select(TurretSubsystem.Alliance.RED,TurretSubsystem.HiveSide.FAR);
        qualify(o,30,1,.8); assertEquals(GoalStateObserver.State.TRANSITIONING,o.state(1.16));
    }
    @Test public void wrongCellZeroPoseNanRangeAndBackFacingCannotQualify() {
        List<TagObservation> invalid=Arrays.asList(tag(34,1.25,30,mount.cameraPitch),
                new TagObservation(30,0,0,0,0,0,0,1),new TagObservation(30,0,0,2,Double.NaN,0,0,1),
                new TagObservation(30,0,0,20,Math.PI,0,0,1),new TagObservation(30,0,0,2,0,0,0,1));
        for(TagObservation tag:invalid) {
            GoalStateObserver o=observer(30);
            for(int i=0;i<5;i++) o.update(1+i*.05,1+i*.05,1+i*.05,Collections.singletonList(tag),true);
            assertFalse(o.raised(1.2));
        }
    }
    @Test public void planeCorroboratesHeightAndTippingRevokesImmediately() {
        GoalStateObserver o=observer(30); qualify(o,30,1,1.25); assertTrue(o.raised(1.16));
        o.update(1.21,1.2,1.2,Collections.singletonList(tag(30,1.25,55,mount.cameraPitch)),true);
        assertFalse(o.raised(1.21));
        qualify(o,30,1.25,1.25); assertTrue(o.reason(),o.raised(1.41));
        o.update(1.46,1.45,1.45,Collections.singletonList(tag(30,1.20,30,mount.cameraPitch)),true);
        assertFalse(o.raised(1.46));
    }
    @Test public void multipleTagsMustAgreeAndMissingTagLossNeedsReacquisition() {
        GoalStateObserver o=observer(30);
        for(int i=0;i<4;i++) { double t=1+i*.05; o.update(t+.01,t,t,Arrays.asList(tag(30,1.25,30,mount.cameraPitch),tag(31,1.26,30,mount.cameraPitch)),true); }
        assertTrue(o.reason(),o.raised(1.16));
        o.update(1.21,1.2,1.2,Arrays.asList(tag(30,1.25,30,mount.cameraPitch),tag(31,.9,30,mount.cameraPitch)),true);
        assertFalse(o.raised(1.21)); assertTrue(o.reason().contains("Conflicting"));
        qualify(o,30,3,1.25); assertTrue(o.reason(),o.raised(3.16));
        o.update(3.2,4,3.2,Collections.emptyList(),true); assertFalse(o.raised(3.2));
        o.update(3.25,5,3.25,Collections.singletonList(tag(30,1.25,30,mount.cameraPitch)),true); assertFalse(o.raised(3.25));
    }
    @Test public void staleGapSideChangeOutOfOrderAndUnsupportedTiltRevoke() {
        GoalStateObserver o=observer(30); qualify(o,30,1,1.25); assertFalse(o.raised(1.4));
        o.update(1.41,5,1.4,Collections.singletonList(tag(30,1.25,30,mount.cameraPitch)),true); assertFalse(o.raised(1.41));
        qualify(o,30,6,1.25); assertTrue(o.raised(6.16));
        o.select(TurretSubsystem.Alliance.RED,TurretSubsystem.HiveSide.AUDIENCE); assertFalse(o.raised(6.16));
        qualify(o,34,7,1.25); assertTrue(o.raised(7.16));
        o.update(7.17,0,7.16,Collections.singletonList(tag(34,1.25,30,mount.cameraPitch)),true); assertFalse(o.raised(7.17));
        qualify(o,34,8,1.25); o.update(8.17,9,8.16,Collections.singletonList(tag(34,1.25,30,mount.cameraPitch)),false); assertFalse(o.raised(8.17));
    }
    @Test public void cameraPitchChangesProjectionButNotRecoveredHeightOrPlane() {
        for(double pitch:new double[]{0,Math.toRadians(30),Math.toRadians(45)}) {
            ShooterMount m=new ShooterMount(mount.pivot,mount.camera,mount.muzzle,mount.zeroYaw,0,pitch,mount.cameraHeight,true);
            TagObservation t=tag(30,1.25,30,pitch);
            assertTrue(t.usable()); assertEquals(1.25,m.cameraHeight+t.levelPosition(m)[2],1e-9);
            assertEquals(30,t.planeDegrees(m),1e-8);
        }
    }
    @Test public void hysteresisRetainsBoundedNoiseButNotAnOpeningAngleUsedAsTagPlane() {
        GoalStateObserver o=observer(30); qualify(o,30,1,1.25);
        o.update(1.21,1.2,1.2,Collections.singletonList(tag(30,1.27,40,mount.cameraPitch)),true);
        // A 10-degree abrupt plane move is treated as motion, even within static retention bounds.
        assertFalse(o.raised(1.21));
        qualify(o,30,3,1.25);
        o.update(3.21,4,3.2,Collections.singletonList(tag(30,1.255,31,mount.cameraPitch)),true); assertTrue(o.reason(),o.raised(3.21));
        o.update(3.26,5,3.25,Collections.singletonList(tag(30,1.25,60,mount.cameraPitch)),true); assertFalse(o.raised(3.26));
    }
    @Test public void differentTagsAndCameraPosesReprojectSameOpening() {
        // Synthetic verified transform, independent of unverified installed reference geometry.
        OpeningGeometry g=new OpeningGeometry("synthetic",true,.2,-.15);
        ShooterMount m=new ShooterMount(new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),0,0,0,.335,true);
        for(double heading:new double[]{0,.4,-.7}) for(int id:new int[]{30,31,32,33,34,38,42,45}) {
            double cameraX=.1,cameraY=-.2, goalX=2,goalY=.3;
            double forward=(goalX-cameraX)*Math.cos(heading)+(goalY-cameraY)*Math.sin(heading);
            double left=-(goalX-cameraX)*Math.sin(heading)+(goalY-cameraY)*Math.cos(heading);
            // Rx(-120deg), independently expanded R * [offsetX,.2,-.15].
            double dx=-OpeningGeometry.tagOffsetMeters(id), dy=-.5*.2-Math.sin(Math.toRadians(-120))*(-.15);
            double dz=Math.sin(Math.toRadians(-120))*.2-.5*(-.15);
            TagObservation t=new TagObservation(id,-left-dx,-(1.5-.335)-dy,forward-dz,Math.toRadians(-120),0,0,1);
            double[] p=g.opening(t,m,pose(1,cameraX,cameraY,heading,0,0,0),0);
            assertNotNull(p); assertEquals(goalX,p[0],1e-9); assertEquals(goalY,p[1],1e-9); assertEquals(1.5,p[2],1e-9);
        }
        assertNull(new OpeningGeometry("unverified",false,.2,-.15).opening(tag(30,1.25,30,0),m,pose(1,0,0,0,0,0,0),0));
    }
    @Test public void observedOpeningPlannerAndFeedTraceRequalifyAfterLossAndRespectManualInhibit() {
        ShooterMount m=new ShooterMount(new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),0,0,mount.cameraPitch,.335,true);
        OpeningGeometry geometry=new OpeningGeometry("synthetic",true,.2,-Math.sqrt(.03));
        GoalStateObserver observer=new GoalStateObserver(m); observer.select(TurretSubsystem.Alliance.RED,TurretSubsystem.HiveSide.FAR);
        ShotPlanner planner=planner(m); FiringCoordinator firing=ShootingConfiguration.firing(); TransferController transfer=new TransferController(1,2);
        boolean initialFeed=false,recoveredFeed=false,manualFeed=false;
        // Each visible/manual interval allows readiness, the command-to-feed delay and feeding.
        for(int i=0;i<320;i++) {
            // Use the same tick scale so rounding cannot put capture slightly in the future.
            double now=1+i*.01, capture=1+(i-i%5)*.01;
            boolean visible=i<110 || i>=195, manual=i>=115&&i<190, inhibit=i>=300;
            TagObservation t=tag(30,1.25,30,m.cameraPitch);
            observer.update(now,i/5,capture,visible?Collections.singletonList(t):Collections.emptyList(),true);
            MotionSample local=new MotionSample(now,0,0,0,0,0,0,0,.03,true,false,"local");
            double[] opening=geometry.opening(t,m,local,0); assertNotNull(opening);
            GoalTarget target=new GoalTarget(TurretSubsystem.Alliance.RED,TurretSubsystem.HiveSide.FAR,GoalTarget.HiveState.RAISED,
                    opening[0],opening[1],opening[2],true,"synthetic",new GoalTarget.Tag(30,2,.1651,1.25)).inLocalFrame();
            ShotPlanner.Solution s=planner.solve(now,local,target,profile());
            firing.update(now,true,manual,inhibit,false,s.valid&&observer.raised(now),true,true,observer.reason());
            transfer.update(now,false,firing.feeding(),true);
            if(i<110) initialFeed|=firing.feeding();
            if(i>=195&&i<300) recoveredFeed|=firing.feeding();
            manualFeed|=manual&&firing.feeding();
            if(i>=110&&i<115 || i>=190&&i<207 || inhibit) { assertFalse(firing.gateOpen()); assertEquals(0,transfer.stage1(),0); }
            if(firing.feeding()) { assertTrue(firing.gateOpen()); assertEquals(1,transfer.stage1(),0); }
        }
        assertTrue("Initial qualified feed", initialFeed);
        assertTrue("Feed after visibility/readiness requalification", recoveredFeed);
        assertTrue("Manual override feeds after gate travel", manualFeed);
    }

    @Test public void sameHeightAndPlaneCannotHideConflictingClusterPositions() {
        GoalStateObserver o=observer(30); qualify(o,30,1,1.25); assertTrue(o.raised(1.16));
        TagObservation t=tag(31,1.25,30,mount.cameraPitch);
        TagObservation conflict=new TagObservation(31,t.x+.5,t.y,t.z,t.roll,t.pitch,t.yaw,t.area);
        o.update(1.21,1.2,1.2,Arrays.asList(tag(30,1.25,30,mount.cameraPitch),conflict),true);
        assertFalse(o.raised(1.21)); assertTrue(o.reason().contains("cluster"));
    }

    @Test public void missingOrInvalidPayloadRevokesEvenWhenFrameCounterDoesNotAdvance() {
        GoalStateObserver o=observer(30); qualify(o,30,1,1.25); assertTrue(o.raised(1.16));
        o.update(1.17,1.15,1.15,Collections.emptyList(),true); assertFalse(o.raised(1.17));
        qualify(o,30,2,1.25); assertTrue(o.raised(2.16));
        o.update(2.17,2.15,2.15,Collections.singletonList(new TagObservation(30,0,0,0,0,0,0,1)),true);
        assertFalse(o.raised(2.17));
    }

    @Test public void telemetryStateReadPreservesSpecificRejectedPoseBeforeFirstAcceptance() {
        GoalStateObserver o=observer(30);
        TagObservation backward=new TagObservation(30,.186,-.015,1.113,0,0,0,.007);
        o.update(1.01,1,1,Collections.singletonList(backward),true);
        assertEquals(GoalStateObserver.State.UNKNOWN,o.state(1.01));
        assertTrue(o.reason(),o.reason().contains("Tag 30: Facing check failed"));
        assertEquals(Double.POSITIVE_INFINITY,o.age(1.01),0);
        assertEquals(GoalStateObserver.State.UNKNOWN,o.state(1.02));
        assertTrue(o.reason(),o.reason().contains("Facing check failed"));
    }

    @Test public void rejectedFreshPoseReasonSurvivesAnOldAcceptedCapture() {
        GoalStateObserver o=observer(30); qualify(o,30,1,1.25);
        assertTrue(o.raised(1.16));
        TagObservation invalid=new TagObservation(30,0,0,2,Double.NaN,0,0,1);
        o.update(2.01,2,2,Collections.singletonList(invalid),true);
        assertEquals(GoalStateObserver.State.UNKNOWN,o.state(2.01));
        assertTrue(o.reason(),o.reason().contains("Non-finite"));
        qualify(o,30,3,1.25);
        assertTrue(o.reason(),o.raised(3.16));
        assertFalse(o.raised(3.4));
        assertEquals("Selected cell lost / stale",o.reason());
    }

}
