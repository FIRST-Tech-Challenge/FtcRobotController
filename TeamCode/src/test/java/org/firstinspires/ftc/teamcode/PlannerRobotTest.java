package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.horizon36596.simloop.fakehardware.*;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

/**
 * PlannerRobot on independent fake hardware with a simulated clock: the planner steps run TeleOp's shared shot
 * pipeline (volley, shot, intake, transfer, gate), missing optional hardware is skipped, and alliance mirroring of
 * named cells matches Zenith. No physical motion or calibration claim.
 */
public class PlannerRobotTest {
    private double now = 100;
    private FakeHardwareMap map;
    private final FakeDigitalChannel full = new FakeDigitalChannel();
    private PlannerRobot robot;

    private FakeMotor motor(String name) { FakeMotor m = new FakeMotor(2800, 8, 28); map.register(name, m); return m; }

    private void hardware(boolean optional) {
        map = new FakeHardwareMap();
        motor("intake_motor"); motor("transfer_motor_1"); motor("transfer_motor_2"); motor("turret_motor");
        for (String n : new String[]{"leftfront", "rightfront", "leftback", "rightback"}) motor(n);
        map.register("distance_sensor", full); full.setSimulatedInputState(false);
        map.register("pinpoint", new FakeGoBildaPinpoint());
        map.register("turret_encoder", new FakeExternalEncoder(6000, TurretSubsystem.TURRET_TICKS_PER_REV));
        for (String n : new String[]{"gate_servo", "turret_servo_1", "turret_servo_2", "turret_pitch"}) map.register(n, new FakeServo());
        if (optional) {
            map.register("depollinator", new FakeServo());
            for (String n : new String[]{"led_intake", "led_shooter", "led_status"}) map.register(n, new FakeServo());
        }
        robot = new PlannerRobot(map, new PedroDrive(map), TurretSubsystem.Alliance.RED, false, false, () -> now);
    }

    @Before public void setup() { hardware(true); }

    /** One exported-OpMode loop with a synthetic fresh, field-referenced pose (meters). */
    private void tick(double seconds) {
        now += seconds;
        map.updateAll(seconds);
        MotionSample pose = ControllerFixtures.pose(now, -.6, -.9, 0, 0, 0, 0).withReference(1);
        robot.readSensors(now, pose);
        robot.apply(now, pose);
    }
    private void home() {
        robot.rehomeTurret();
        for (int i = 0; i < 150 && !robot.turretHomed(); i++) tick(.02);
        assertTrue(robot.turret.getHomingStatus(), robot.turretHomed());
    }
    private double power(String name) { return map.get(DcMotorEx.class, name).getPower(); }

    @Test public void volleyRunsTeleOpsAimReadyFeedSequenceThenSpinsDown() {
        home();
        double started = now;
        robot.startVolley(new Pose(TurretSubsystem.RED_HIVE_PIVOT_X_IN, -TurretSubsystem.HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN, 0),
                1.0, 3.0, PlannerVolley.TimeoutAction.FIRE, PlannerRobot.After.OFF);
        tick(.02);
        assertEquals("TeleOp's exact cell target", TeleOpFieldReference.target(TurretSubsystem.Alliance.RED,
                TurretSubsystem.HiveSide.AUDIENCE).identity(), robot.shots.target().identity());
        assertEquals(PlannerVolley.Phase.AIMING, robot.volleyPhase());
        assertTrue("shot table solution for the cell", robot.shotSolution().valid);
        assertEquals("RPM from ShootingConfiguration.PROFILE", robot.shotSolution().rpm, robot.turret.getTargetShooterRpm(), 1e-6);
        assertFalse("never feeds before ready", robot.gate.isOpenCommanded());
        // The fake turret encoder does not follow the yaw servos and the untuned bang-bang loop hunts on the fake
        // flywheel, so TeleOp's readiness is never met here: the volley must hold fire for the whole ready timeout,
        // then FIRE with the homed turret (the ready path itself: PlannerVolleyTest, FiringCoordinatorTest, TeleOp sims).
        double firstFeed = Double.NaN;
        boolean fed = false;
        for (int i = 0; i < 250 && robot.isShooting(); i++) {
            tick(.02);
            if (robot.transfer.isFeeding()) {
                if (!fed) firstFeed = now;
                fed = true;
                assertTrue(robot.gate.isOpenCommanded());
                assertEquals(1, power("transfer_motor_1"), 0); assertEquals(1, power("transfer_motor_2"), 0);
                assertEquals("intake pauses while shooting", 0, power("intake_motor"), 0);
            }
        }
        assertTrue("the volley fed", fed);
        assertTrue("no feed before the ready timeout without readiness", firstFeed - started >= 3.0 - 1e-9);
        assertEquals("fed before READY (timeout)", robot.shotNote);
        assertEquals(PlannerVolley.Phase.DONE, robot.volleyPhase());
        assertTrue(robot.volleyDone());
        tick(.02);
        assertFalse(robot.gate.isOpenCommanded());
        assertFalse(robot.transfer.isFeeding());
        assertEquals("After OFF spins the flywheel down", 0, robot.turret.getTargetShooterRpm(), 0);
        assertEquals(PlannerRobot.Flywheel.OFF, robot.flywheel());
    }

    @Test public void unhomedVolleySkipsInsteadOfFiringBlind() {
        robot.startVolley(null, 1.0, .5, PlannerVolley.TimeoutAction.FIRE, PlannerRobot.After.KEEP);
        robot.turret.stop();   // invalidate the INIT home so the turret yaw is unknown
        for (int i = 0; i < 40; i++) tick(.02);
        assertTrue(robot.volleySkipped());
        assertFalse(robot.transfer.isFeeding());
        assertFalse(robot.gate.isOpenCommanded());
    }

    @Test public void manualShotFeedsAtOnceForTheFeedTime() {
        robot.startShot(false, .5, 3, PlannerVolley.TimeoutAction.FIRE, PlannerRobot.After.KEEP);
        tick(.02);
        assertTrue("TeleOp GP1 A opens the gate at once", robot.gate.isOpenCommanded());
        for (int i = 0; i < 30 && robot.isShooting(); i++) tick(.02);
        assertTrue(robot.volleyDone());
        tick(.02);
        assertFalse(robot.gate.isOpenCommanded());
    }

    @Test public void intakeUntilFullUsesTheTeleOpFullQueueRule() {
        robot.setIntake(PlannerRobot.IntakeMode.IN, 0, true);
        tick(.02);
        assertEquals(1, power("intake_motor"), 0);
        full.setSimulatedInputState(true);
        tick(.02); tick(.4);
        assertFalse("full needs 0.5 s HIGH", robot.queueFull());
        assertEquals(1, power("intake_motor"), 0);
        tick(.11);
        assertTrue(robot.queueFull());
        tick(.02);
        assertEquals("IntakeQueueController.FULL_QUEUE_INTAKE_POWER", IntakeQueueController.FULL_QUEUE_INTAKE_POWER, power("intake_motor"), 0);
        assertEquals("off once full", PlannerRobot.IntakeMode.OFF, robot.intakeMode());
    }

    @Test public void timedIntakeTurnsItselfOffWhileDriving() {
        robot.setIntake(PlannerRobot.IntakeMode.REVERSE, .3, false);
        tick(.02);
        assertEquals(-1, power("intake_motor"), 0);
        for (int i = 0; i < 20; i++) tick(.02);
        assertEquals(PlannerRobot.IntakeMode.OFF, robot.intakeMode());
        assertEquals(0, power("intake_motor"), 0);
    }

    @Test public void transferHoldPausesLoadingAndGateHoldOpens() {
        tick(.02);
        assertEquals("TeleOp loads stage 1 until full", 1, power("transfer_motor_1"), 0);
        robot.holdTransfer(true); tick(.02);
        assertEquals(0, power("transfer_motor_1"), 0);
        robot.holdTransfer(false); robot.gateOpen(true); tick(.02);
        assertTrue(robot.gate.isOpenCommanded());
        assertEquals(1, power("transfer_motor_1"), 0);
        robot.gateOpen(false); tick(.02);
        assertFalse(robot.gate.isOpenCommanded());
    }

    @Test public void stopAllStopsEveryOutput() {
        robot.setIntake(1); robot.spin(true); robot.setRpmManual(2500); tick(.02);
        assertEquals(2500, robot.turret.getTargetShooterRpm(), 0);
        robot.stopAll(); tick(.02);
        assertEquals(0, power("intake_motor"), 0);
        assertEquals(0, power("transfer_motor_1"), 0);
        assertEquals(0, robot.turret.getTargetShooterRpm(), 0);
        assertFalse(robot.gate.isOpenCommanded());
    }

    @Test public void missingOptionalHardwareIsSkipped() {
        hardware(false);
        robot.deployDepollinator();
        assertEquals("SKIPPED: depollinator not configured", robot.shotNote);
        assertTrue(robot.depollinatorSettled());
        robot.raiseSlide();
        assertTrue(robot.slideSettled());
        assertFalse(robot.slideFault());
        robot.startCollect();
        assertTrue(robot.collectDone());
        robot.startRelocalize(2);
        assertTrue("no Limelight / tag relocalization: skipped", robot.relocalizeDone());
        robot.showLights(RobotStatusLights.Color.GREEN, 1);
        tick(.02);
        List<String> lines = new ArrayList<>();
        robot.addTelemetry(telemetry(lines));
        assertTrue(String.join("\n", lines), String.join("\n", lines).contains("depollinator, linear slide"));
    }

    @Test public void namedCellsSwapUnderTheAllianceMirrorLikeZenith() {
        assertEquals(TurretSubsystem.HiveSide.AUDIENCE, PlannerRobot.authoredSide(TurretSubsystem.HiveSide.AUDIENCE, false));
        assertEquals(TurretSubsystem.HiveSide.FAR, PlannerRobot.authoredSide(TurretSubsystem.HiveSide.AUDIENCE, true));
        // The planner mirrors aim points through the field center; red audience becomes blue far, Zenith's rule.
        Pose redAudience = new Pose(TurretSubsystem.RED_HIVE_PIVOT_X_IN, -TurretSubsystem.HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN, 0);
        GoalTarget planned = PlannerRobot.targetFor(redAudience);
        GoalTarget mirrored = PlannerRobot.targetFor(new Pose(-redAudience.x(), -redAudience.y(), 0));
        assertEquals(TurretSubsystem.Alliance.RED, planned.alliance);
        assertEquals(TurretSubsystem.HiveSide.AUDIENCE, planned.side);
        assertEquals(TurretSubsystem.Alliance.BLUE, mirrored.alliance);
        assertEquals(PlannerRobot.authoredSide(planned.side, true), mirrored.side);
        assertEquals(TeleOpFieldReference.target(TurretSubsystem.Alliance.BLUE, TurretSubsystem.HiveSide.FAR).identity(), mirrored.identity());
    }

    @Test public void customAimPointKeepsTheNearestCellButItsOwnPosition() {
        GoalTarget custom = PlannerRobot.targetFor(new Pose(-20, 10, 0));
        assertEquals(TurretSubsystem.Alliance.RED, custom.alliance);
        assertEquals(TurretSubsystem.HiveSide.FAR, custom.side);
        assertEquals(-20 * PlannerRobot.METERS_PER_INCH, custom.x, 1e-9);
        assertEquals(10 * PlannerRobot.METERS_PER_INCH, custom.y, 1e-9);
        assertTrue(custom.scoreable());
    }

    @Test public void nearestCellFollowsTheRobotsHalfOfTheField() {
        home();
        robot.aimAtNearestCell();
        robot.setTracking(true);
        tick(.02);
        assertEquals(TurretSubsystem.HiveSide.AUDIENCE, robot.shots.target().side);
        assertTrue(robot.shots.followsNearestCell());
        MotionSample far = ControllerFixtures.pose(now + .02, -.6, .9, 0, 0, 0, 0).withReference(1);
        now += .02; map.updateAll(.02);
        robot.readSensors(now, far); robot.apply(now, far);
        assertEquals("TeleOp rule: crossing the midline switches cells", TurretSubsystem.HiveSide.FAR, robot.shots.target().side);
        robot.aimAt(new Pose(TurretSubsystem.RED_HIVE_PIVOT_X_IN, -TurretSubsystem.HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN, 0));
        assertFalse(robot.shots.followsNearestCell());
    }

    @Test public void trimsAreClippedToTeleOpBounds() {
        AutoAimAdjustments a = new AutoAimAdjustments();
        a.set(50, -50, 9000);
        assertEquals(AutoAimAdjustments.MAX_YAW_DEG, a.yawDeg(), 0);
        assertEquals(-AutoAimAdjustments.MAX_HOOD_DEG, a.hoodDeg(), 0);
        assertEquals(AutoAimAdjustments.MAX_RPM, a.rpm(), 0);
        a.set(Double.NaN, 1, 2);
        assertEquals(0, a.yawDeg(), 0);
    }

    private static Telemetry telemetry(List<String> lines) {
        return (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(), new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("addData") && args != null) {
                        StringBuilder b = new StringBuilder(String.valueOf(args[0]));
                        for (int i = 1; i < args.length; i++) b.append(" ").append(args[i] instanceof Object[]
                                ? java.util.Arrays.toString((Object[]) args[i]) : String.valueOf(args[i]));
                        lines.add(b.toString());
                    }
                    Class<?> r = method.getReturnType();
                    if (r == boolean.class) return Boolean.FALSE;
                    if (r == int.class) return Integer.valueOf(0);
                    if (r == double.class) return Double.valueOf(0);
                    return null;
                });
    }
}
