package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import java.util.*;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;

public class HardwareReviewRegressionTest {
    @Test public void missingNamedServoNeverSubstitutesAnInventoryDeviceOnAnotherHub() {
        Object wrongHub=new Object(), intended=new Object(); Map<String,Object> names=new HashMap<>();
        names.put("unrelated_servo_port_1",wrongHub);
        IllegalArgumentException error=assertThrows(IllegalArgumentException.class,()->NamedHardware.require(names::get,
                ()->"unrelated_servo_port_1 [Control Hub USB 1, port 1]","turret_servo_1","turretServo1"));
        assertTrue(error.getMessage().contains("turret_servo_1")); assertTrue(error.getMessage().contains("Control Hub"));
        names.put("turretServo1",intended); assertSame(intended,NamedHardware.require(names::get,()->"unused","turret_servo_1","turretServo1"));
    }
    @Test public void relativeZeroAndFaultCannotRequestHomeWithoutAnExplicitRetry() {
        TurretHomingController h=new TurretHomingController(1,.3,3,.2);
        assertFalse(h.commandsHome()); h.start(1); assertTrue(h.commandsHome());
        h.update(2,10,0); assertTrue(h.commandsHome()); h.update(2.31,10,0); assertTrue(h.ready()); assertFalse(h.commandsHome());
        h.invalidate(); h.update(2.5,100,30); assertFalse(h.ready()); assertFalse(h.commandsHome());
        h.start(3); h.update(6.1,100,30); assertEquals(TurretHomingController.State.FAULT,h.state()); assertFalse(h.commandsHome());
        h.start(7); assertTrue(h.commandsHome()); h.update(8,100,30); h.update(8.31,100,30); assertTrue(h.ready());
    }
    @Test public void delayedTrackingUsesCapturedTurretAndChassisThenCurrentChassis() {
        PoseHistory h=new PoseHistory(); ShooterMount m=new ShooterMount(new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),0,0,0,.3,true);
        h.add(pose(1,0,0,.1,0,0,0),0);
        for(int i=1;i<=10;i++) h.add(pose(1+i*.01,0,0,.1+i*.01,0,0,0),i*.01);
        // At capture: chassis .1 + turret 0 + target error +.1. Current chassis .2 => target yaw 0.
        assertEquals(m.cameraYaw,h.delayedYaw(1.1,1,-.1,m),1e-9);
        assertEquals(.2+m.cameraYaw,h.delayedYaw(1.1,1.1,-.1,m),1e-9);
        assertTrue(Double.isNaN(h.delayedYaw(2,1,-.1,m)));
        assertTrue(Double.isNaN(h.delayedYaw(1.1,.8,-.1,m)));
    }
    @Test public void entireMatchWithoutCameraDoesNotTimeOutOdometryAimingOrFeeding() {
        PoseHistory h = new PoseHistory(); ShotPlanner planner = planner(mount());
        FiringCoordinator firing = ShootingConfiguration.firing();
        boolean fedNearEnd = false;
        for (int i = 0; i <= 9000; i++) {
            double now = i * .02;
            h.add(pose(now, 0, 0, 0, .2, 0, 0), 0);
            MotionSample current = h.current(now);
            assertEquals(.02, current.uncertainty, 0);
            ShotPlanner.Solution s = planner.solve(now, current, goal(2,0), profile());
            assertTrue(s.reason, s.valid);
            firing.update(now, true, false, false, false, s.valid, true, true, s.reason);
            if (now > 178 && firing.feeding()) fedNearEnd = true;
        }
        assertTrue(fedNearEnd);
        assertFalse(planner.solve(181, h.current(181), goal(2,0), profile()).valid);
    }
    @Test public void acceptedVisionKeepsQualityEstimateWithoutInventingDrift() {
        PoseHistory h=new PoseHistory(); GoalTarget g=goal(2,0); ShooterMount m=mount();
        double ty=Math.atan2(.7,2)-m.cameraPitch;
        for(int i=0;i<=18000;i++) {
            double now=1+i*.01;
            MotionSample p=new MotionSample(now,0,0,0,0,0,0,0,.02+i*.00001,true,true,"field");
            h.add(p,0);
            if(i>10 && i%5==0) {
                assertTrue(h.reason(),h.observe(now,i,now-.05,30,0,ty,g,m));
                assertTrue(h.current(now).uncertainty<.15);
            }
        }
        double before=h.current(181).uncertainty;
        assertEquals(before,h.current(183).uncertainty,0); assertFalse(h.targetFresh(183));
    }
    @Test public void metricFusionRespectsJumpLimitAndDoesNotRewriteMeasuredVelocity() {
        PoseHistory h=new PoseHistory(); ShooterMount m=mount(); GoalTarget g=goal(2,0);
        for(int i=0;i<20;i++) h.add(pose(1+i*.01,.1,0,0,.2,0,0),0);
        TagObservation t=TagFixtures.seen(g,30,pose(1.1,0,0,0,0,0,0),0,m);
        boolean accepted=h.observe3d(1.19,1,1.1,t,g,m);
        assertTrue(h.reason(),accepted);
        assertTrue(h.current(1.19).x<.1); assertEquals(.2,h.current(1.19).vx,0);
        assertTrue(h.current(1.19).uncertainty>=.025);
        TagObservation jump=new TagObservation(30,t.x,t.y,t.z+2,t.roll,t.pitch,t.yaw,1);
        assertFalse(h.observe3d(1.19,2,1.1,jump,g,m));
    }
    @Test public void localTargetCanAimWithoutInventingFieldReference() {
        MotionSample local=new MotionSample(1,0,0,0,0,0,0,0,.03,true,false,"local startup");
        GoalTarget field=goal(2,0), relative=field.inLocalFrame();
        assertFalse(planner(mount()).solve(1,local,field,profile()).valid);
        ShotPlanner.Solution s=planner(mount()).solve(1,local,relative,profile());
        assertTrue(s.reason,s.valid); assertFalse(local.fieldReferenced); assertEquals(GoalTarget.Frame.LOCAL_ODOMETRY,relative.frame);
    }
    @Test public void transferTimingComesFromConfigurationAndFullWaitsForFeed() {
        TransferController c=new TransferController(.6,.4);
        c.update(1,true,false,false); assertEquals(.6,c.stage1(),0);
        c.update(1.39,true,false,false); assertFalse(c.full());
        c.update(1.41,true,false,false); assertTrue(c.full()); assertEquals(.6,c.stage1(),0);
        c.update(1.42,true,false,false); assertTrue(c.full()); assertEquals(.6,c.stage1(),0);
        c.update(1.43,true,false,true); assertEquals(0,c.stage1(),0);
        c.update(1.44,true,false,false); assertEquals(0,c.stage1(),0); // Pause cancels run-on.
        c.update(1.5,true,true,true); assertEquals(.6,c.stage1(),0);
        c.stop(); assertEquals(0,c.stage1(),0); assertFalse(c.full());
    }
    @Test public void traceNeverFeedsUntilGateDelayAndLossManualInhibitAndShutdownRemainDistinct() {
        FiringCoordinator f=ShootingConfiguration.firing(); TransferController c=new TransferController(1,2);
        c.stop(); assertEquals(0,c.stage1(),0);
        double opened=Double.NaN; boolean sawFeed=false;
        for(int i=0;i<=200;i++) {
            double t=i*.01;
            boolean manual=t>=1.1, inhibit=t>=1.8, raised=t<.9;
            f.update(t,true,manual,inhibit,false,raised,true,true,"Goal UNKNOWN / moving");
            if(f.gateOpen() && Double.isNaN(opened)) opened=t;
            if(!f.gateOpen()) opened=Double.NaN;
            boolean actualFeed=f.feeding() && !Double.isNaN(opened) && t-opened>=GateSubsystem.FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS;
            c.update(t,i%3!=0,actualFeed,true);
            if(actualFeed) { sawFeed=true; assertTrue(f.gateOpen()); assertTrue(t-opened>=GateSubsystem.FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS); assertEquals(1,c.stage1(),0); }
            else assertEquals(0,c.stage1(),0); // Paused outside the feed.
            if(t>=.9 && t<1.1 || inhibit) { assertFalse(f.gateOpen()); assertEquals(0,c.stage1(),0); }
        }
        assertTrue(sawFeed);
        try { throw new IllegalStateException("simulated loop exception"); }
        catch(IllegalStateException expected) { /* adapter finally cleanup */ }
        finally { f.stop(2.1); c.stop(); }
        assertFalse(f.gateOpen()); assertFalse(f.feeding()); assertEquals(0,c.stage1(),0);
    }
    @Test public void referenceResetInvalidatesCorrectionsHistory() {
        PoseHistory history=new PoseHistory();
        for(int i=0;i<30;i++) history.add(pose(1+i*.01,0,0,0,0,0,0).withReference(1),0);
        history.acceptRelative(1.2,.03); assertTrue(history.targetFresh(1.29));
        history.add(pose(1.3,2,3,0,0,0,0).withReference(2),0);
        assertNull(history.at(1.2)); assertFalse(history.targetFresh(1.3)); assertEquals(2,history.current(1.3).referenceId);
    }
    @Test public void shutdownAttemptsAllDevicesAndPreservesFirstFailure() {
        java.util.List<String> called=new java.util.ArrayList<>();
        IllegalStateException failure=new IllegalStateException("hub disconnected");
        RuntimeException thrown=assertThrows(RuntimeException.class,()->RobotShutdown.stopAll(
                ()->{called.add("drive"); throw failure;},()->called.add("intake"),
                ()->{called.add("transfer"); throw new IllegalArgumentException("second device");},()->called.add("gate")));
        assertSame(failure,thrown); assertEquals(java.util.Arrays.asList("drive","intake","transfer","gate"),called);
        assertEquals(1,thrown.getSuppressed().length);
    }
    @Test public void metricDelayedTrackingCompensatesChassisTranslationAndAngleOnlyTrackingRejectsParallax() {
        ShooterMount m=new ShooterMount(new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),new RobotMath.Vec(0,0),0,0,0,.335,true);
        PoseHistory h=new PoseHistory(); h.add(pose(1,0,0,0,0,0,0),0);
        for(int i=1;i<=10;i++) h.add(pose(1+i*.01,0,i*.01,0,0,0,0),i*.01);
        TagObservation tag=new TagObservation(30,0,-.915,2,Math.PI,0,0,1);
        assertEquals(Math.atan2(-.1,2),h.delayedYaw3d(1.1,1,tag,m),1e-9);
        assertTrue(Double.isNaN(h.delayedYaw(1.1,1,0,m)));
    }

    @Test public void VelocityAndAccelerationDoNotGateCurrentPoseAiming() {
        ShotPlanner p=planner(mount()); ShotProfile table=new ShotProfile("synthetic",true,
                new ShotProfile.Row(.2,1,3000),new ShotProfile.Row(5,10,3400));
        ShotPlanner.Solution result=null;
        for(int i=0;i<40;i++) {
            double t=1+i*.01;
            MotionSample noisy=new MotionSample(t,0,0,0,i%2==0?.015:-.015,0,.01,3,2,.03,true,true,"bounded velocity noise");
            result=p.solve(t,noisy,goal(2,0),table);
        }
        assertNotNull(result); assertTrue(result.reason,result.valid);
        MotionSample moving=new MotionSample(1.4,0,0,0,.1,0,.1,3,2,.03,true,true,"actual motion");
        assertTrue(p.solve(1.4,moving,goal(2,0),table).valid);
    }
    @Test public void acceptedFusionKeepsUncorrectedResidualInUncertaintyBudget() {
        PoseHistory h=new PoseHistory(); ShooterMount m=mount();
        for(int i=0;i<20;i++) h.add(pose(1+i*.01,.4,0,0,0,0,0),0);
        assertTrue(h.observe(1.19,1,1.1,30,0,Math.atan2(.7,2)-m.cameraPitch,goal(2,0),m));
        assertTrue(h.current(1.19).uncertainty>.25); // .35 gain leaves .26 m of the .4 m innovation.
        assertTrue(new ShotPlanner(m).solve(1.19,h.current(1.19),goal(2,0),profile()).valid);
    }

}
