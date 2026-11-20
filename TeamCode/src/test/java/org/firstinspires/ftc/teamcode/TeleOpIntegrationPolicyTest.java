package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.ControllerFixtures.*;
import static org.firstinspires.ftc.teamcode.RobotStatusLights.Color.*;

public class TeleOpIntegrationPolicyTest {
    @Test public void firingPauseStopsAutomaticIntakeButRetainsExplicitOverrides() {
        IntakeQueueController c = new IntakeQueueController();
        for (boolean full : new boolean[]{false, true}) {
            assertEquals(0, c.update(full, false, false, true), 0);
            assertEquals(1, c.update(full, true, false, true), 0);
            assertEquals(-1, c.update(full, false, true, true), 0);
            assertEquals(0, c.update(full, true, true, true), 0);
        }
        assertEquals(1, c.update(false, false, false, false), 0);
        assertEquals(0, c.update(true, false, false, false), 0);
    }
    @Test public void trimsRemainBoundedAndResetWithoutChangingCalibrationValidity() {
        AutoAimAdjustments a = new AutoAimAdjustments();
        for (int i = 0; i < 100; i++) a.adjust(1, .1, 1, 1);
        assertEquals(-10, a.yawDeg(), 0); assertEquals(5, a.hoodDeg(), 0); assertEquals(500, a.rpm(), 0);
        ShotPlanner.Solution base = planner(mount()).solve(1, pose(1,0,0,0,0,0,0), goal(2,0), profile());
        ShotPlanner.Solution adjusted = base.withOffsets(a.yawDeg(), a.hoodDeg(), a.rpm());
        assertTrue(adjusted.valid);
        assertEquals(base.yaw - Math.toRadians(10), adjusted.yaw, 1e-9);
        assertEquals(base.hoodDeg + 5, adjusted.hoodDeg, 1e-9);
        assertEquals(base.rpm + 500, adjusted.rpm, 1e-9);
        assertFalse(base.inhibit("Unmeasured").withOffsets(1, 1, 50).valid);
        assertFalse(ShotPlanner.Solution.invalid("Uncalibrated").withOffsets(0,0,0).aimable);
        for (double[] trim : new double[][]{{180,0,0},{0,40,0},{0,0,-6000},{0,0,6000},{Double.NaN,0,0}})
            assertFalse(base.withOffsets(trim[0],trim[1],trim[2]).valid);
        a.reset(); assertEquals(0, a.yawDeg(), 0); assertEquals(0, a.hoodDeg(), 0); assertEquals(0, a.rpm(), 0);
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
    /** One competition-order loop with RT held: sensors, field-referenced Pinpoint sample, aim, same-loop yaw write. */
    private static void aimLoop(double[] clock, TurretSubsystem turret, ShotCoordinator shots, double x, double y) {
        clock[0] += .02; turret.update(clock[0]);
        shots.update(clock[0], new MotionSample(clock[0], x, y, 0, 0, 0, 0, 0, .02, true, true, "test"), true, false, false, false);
        turret.applyYawCommand(clock[0]);
    }
    private static ShotCoordinator teleOpShots(TurretSubsystem turret, ShotProfile profile) {
        return new ShotCoordinator(turret, TeleOpFieldReference.target(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.AUDIENCE),
                ShootingConfiguration.MOUNT, profile, CompetitionFeedController.firing(), ShotCoordinator.AimMode.TELEOP_ODOMETRY);
    }
    private static ShotProfile constantHood(double hoodDeg) {
        return new ShotProfile(ShootingConfiguration.CALIBRATION_GEOMETRY_ID, true,
                new ShotProfile.Row(.2, hoodDeg, 2800), new ShotProfile.Row(5, hoodDeg, 2800));
    }

    @Test public void selectedCellDoesNotChatterWhileDrivingAlongTheMidline() {
        double[] clock = {0};
        TurretSubsystem turret = homedTurret(clock);
        ShotCoordinator shots = teleOpShots(turret, ShootingConfiguration.PROFILE);
        int switches = 0;
        TurretSubsystem.HiveSide side = turret.getActiveHiveSide();
        for (int i = 0; i < 200; i++) {
            aimLoop(clock, turret, shots, -1.5, .03 * Math.sin(i * .7)); // +/-3 cm weave across Y = 0
            if (turret.getActiveHiveSide() != side) { switches++; side = turret.getActiveHiveSide(); }
        }
        assertEquals(0, switches);
        for (int i = 0; i < 5; i++) aimLoop(clock, turret, shots, -1.5, .2);
        assertEquals(TurretSubsystem.HiveSide.FAR, turret.getActiveHiveSide()); // Clearly on the far half: one switch.
    }

    @Test public void hoodAndRpmTrimsAreOnlyAvailableWithACalibratedSolutionAndStayAdjustableOutOfRange() {
        double[] clock = {0};
        TurretSubsystem turret = homedTurret(clock);
        // An uncalibrated profile: auto aim sets yaw only, so GP2 keeps the manual hood/RPM.
        ShotCoordinator uncalibrated = teleOpShots(turret, ShotProfile.example());
        for (int i = 0; i < 5; i++) aimLoop(clock, turret, uncalibrated, -1.5, -.5);
        assertFalse(uncalibrated.solution().aimable);
        assertFalse(uncalibrated.automaticHoodRpmAvailable());
        // A calibrated solution: trims apply, including one that pushes the hood past its travel, so it can be undone.
        ShotCoordinator calibrated = teleOpShots(turret, constantHood(28));
        for (int i = 0; i < 5; i++) aimLoop(clock, turret, calibrated, -1.5, -.5);
        assertTrue(calibrated.solution().reason, calibrated.solution().aimable);
        assertTrue(calibrated.automaticHoodRpmAvailable());
        AutoAimAdjustments trims = new AutoAimAdjustments();
        trims.adjust(0, .02, 10, 0); // +5 deg hood: 33 deg exceeds the 30 deg travel
        calibrated.setAutoAimAdjustments(clock[0], trims);
        aimLoop(clock, turret, calibrated, -1.5, -.5);
        assertFalse(calibrated.solution().aimable);
        assertTrue(calibrated.automaticHoodRpmAvailable());
    }
    @Test public void lightsDistinguishManualBypassInhibitAndAutomaticReadiness() {
        assertEquals(GREEN, RobotStatusLights.shooterColor(0, FiringCoordinator.State.IDLE, false, true, true));
        assertEquals(YELLOW, RobotStatusLights.shooterColor(0, FiringCoordinator.State.PREPARING, false, false, true));
        assertEquals(RED, RobotStatusLights.shooterColor(0, FiringCoordinator.State.INHIBITED, true, true, true));
        assertEquals(VIOLET, RobotStatusLights.shooterColor(.1, FiringCoordinator.State.FEEDING, true, false, true));
        assertEquals(OFF, RobotStatusLights.shooterColor(.3, FiringCoordinator.State.FEEDING, true, false, true));
        assertEquals(VIOLET, RobotStatusLights.shooterColor(.3, FiringCoordinator.State.FEEDING, false, true, true));
        assertEquals(GREEN, RobotStatusLights.intakeColor(true, false, false, -1));
        assertEquals(BLUE, RobotStatusLights.intakeColor(true, true, false, 1));
        assertEquals(OFF, RobotStatusLights.intakeColor(true, true, true, 0));
    }
    @Test public void anyNewFaultAlertsForTwoSecondsAndEachNewFaultRestartsIt() {
        RobotStatusLights.FaultAlert alert = new RobotStatusLights.FaultAlert();
        assertFalse(alert.update(0, false, false));
        assertTrue(alert.update(1, true, false));   // Fault present (also at the first reading).
        assertTrue(alert.update(2.9, true, false));
        assertFalse(alert.update(3.0, true, false)); // 2 s later, although the fault persists.
        assertTrue(alert.update(3.5, true, true));   // A second fault restarts the alert.
        assertEquals(0, alert.elapsed(3.5), 1e-9);
        assertFalse(alert.update(5.5, true, true));
        assertFalse(alert.update(6, false, true));   // A fault clearing never alerts.
        assertTrue(alert.update(7, true, true));     // The same fault returning does.
        RobotStatusLights.FaultAlert initial = new RobotStatusLights.FaultAlert();
        assertTrue(initial.update(0, true));         // Already faulted at the first reading.
    }
}
