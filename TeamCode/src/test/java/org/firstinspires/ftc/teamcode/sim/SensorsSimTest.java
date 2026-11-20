package org.firstinspires.ftc.teamcode.sim;

import org.junit.jupiter.api.Test;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.firstinspires.ftc.teamcode.GoalTarget;
import org.firstinspires.ftc.teamcode.MotionSample;
import org.firstinspires.ftc.teamcode.ShootingConfiguration;
import org.firstinspires.ftc.teamcode.TeleOpFieldReference;
import org.firstinspires.ftc.teamcode.TurretSubsystem;
import static org.junit.jupiter.api.Assertions.*;

class SensorsSimTest {
    @Test void cameraRejectionDiagnosticsAgreeAcrossCameraTestsAutoAimAndBothTeleOps() throws Exception {
        for (String opMode : new String[]{"LimelightTagTestOpMode", "GoalStateTestOpMode",
                "AutoAimTestOpMode", "RedMecanumTeleOp", "BlueMecanumTeleOp"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.forOpMode(opMode)
                    .withCamera(BioBuzzSimHardware.raisedAudienceTags(), false); // Inverted-normal camera.
            // Camera-only OpModes do not initialize yaw servos. Place the unpowered mount
            // at a known home angle so the fixture does not slew it from a default zero command.
            h.yaw1.setPosition(.5); h.yaw2.setPosition(.5);
            TeleOpScenario s = new TeleOpScenario("rejection-diagnostics-"+opMode,opMode,h,
                    .02,160,100,(p,t,a,b) -> {
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                if (opMode.equals("GoalStateTestOpMode")) a.dpad_left = true;
                // Competition TeleOps show camera rejection details on the GP2 Back diagnostics page.
                if (opMode.endsWith("MecanumTeleOp")) b.back = p == OpModePhase.ACTIVE && t >= 1 && t < 4;
            });
            double x = opMode.equals("BlueMecanumTeleOp") ? 12.75 : -12.75;
            s.model.drive.setPose(x,BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES,Math.PI);
            h.pinpoint.attachChassisAt(x*25.4,BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4,Math.PI);
            s.run();
            String text = s.trace.at(OpModePhase.ACTIVE,60).telemetry;
            assertTrue(text.contains("Camera capture age (s)"),s.diagnostic());
            assertTrue(text.contains("R/P/Y (deg), facing"),s.diagnostic());
            assertTrue(text.contains("Facing check failed"),s.diagnostic());
            assertTrue(text.contains("usable false"),s.diagnostic());
            if (opMode.equals("GoalStateTestOpMode")) {
                assertTrue(text.contains("Decision : Tag "),s.diagnostic());
                assertFalse(text.contains("Decision : Selected cell lost / stale"),s.diagnostic());
            } else if (!opMode.equals("LimelightTagTestOpMode")) {
                assertTrue(text.contains("Vision decision : Tag "),s.diagnostic());
                assertFalse(text.contains("Field localization : LOCALIZED"),s.diagnostic());
                if (opMode.endsWith("MecanumTeleOp"))
                    assertEquals(.5,s.trace.at(OpModePhase.ACTIVE,60).gatePosition,s.diagnostic());
                assertFalse(s.trace.at(OpModePhase.ACTIVE,60).feedingThroughOpenGate(),s.diagnostic());
            }
        }
    }

    @Test void bothAllianceCornerRehomesFallBackToCornerWhenRaisedTagsAreBelowTheImage() throws Exception {
        for (String alliance : new String[]{"Red","Blue"}) for (boolean physical : new boolean[]{false,true})
                for (boolean inInit : new boolean[]{false,true}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedCornerRehomeTags(),physical);
            TeleOpScenario s = new TeleOpScenario(alliance+"-corner-vision-physical-"+physical+"-init-"+inInit,alliance+"MecanumTeleOp",
                    h,.02,160,260,(p,t,a,b) -> {
                b.dpad_left = p == OpModePhase.INIT_LOOP ? inInit && t >= 145
                        : p == OpModePhase.ACTIVE && t >= (inInit ? 0 : 20) && t < 55;
                if (p == OpModePhase.ACTIVE && t == 210) h.camera.stop();
            });
            double sign = alliance.equals("Blue") ? 1 : -1;
            double cornerX = sign*(72*.0254-.262481), cornerY = -sign*(72*.0254-.15835);
            double actualX = cornerX-sign*.025, actualY = cornerY+sign*.02;
            double heading = sign > 0 ? Math.PI : 0;
            s.model.drive.setPose(actualX/.0254,actualY/.0254,heading);
            h.pinpoint.attachChassisAt(actualX*1000,actualY*1000,heading);
            s.run();
            java.util.Optional<TeleOpTrace.Sample> refined = s.trace.samples.stream()
                    .filter(v -> v.phase == OpModePhase.ACTIVE && v.tick > 20
                            && v.telemetry.contains("Field rehome refinement : VISION REFINED")).findFirst();
            // Measured 55-deg Limelight: from the corner the raised tags sit about 28 deg below the
            // lens axis, past the ~20.5-deg half image height, so neither convention can refine.
            assertFalse(refined.isPresent(),s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE,200).telemetry.contains("CORNER ONLY: Limelight refinement timed out"),s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE,250).telemetry.contains("Field localization : LOCALIZED"),s.diagnostic());
            assertEquals(0,org.firstinspires.ftc.teamcode.RobotMath.wrap(h.pinpoint.getPosition()
                    .getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)-heading),1e-6); // Fake Pinpoint stores float32 radians.
            for (TeleOpTrace.Sample v : s.trace.samples) assertEquals(.5,v.gatePosition,s.diagnostic());
            CompetitionSimTest.assertStopped(h);
        }
    }

    @Test void syntheticCornerRefinementHoldsAutomaticFeedUntilCornerFallbackCompletes() throws Exception {
        for (String alliance : new String[]{"Red","Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedCornerRehomeTags(),true);
            TeleOpScenario s = new TeleOpScenario(alliance+"-SYNTHETIC-corner-feed","sim.Synthetic"+alliance+"TeleOp",
                    h,.02,160,240,(p,t,a,b) -> {
                boolean requested = p == OpModePhase.ACTIVE && t >= 20;
                b.dpad_left = requested && t < 30;
                a.right_trigger = requested ? 1 : 0;
                a.left_trigger = requested ? 1 : 0;
            });
            double sign = alliance.equals("Blue") ? 1 : -1;
            double x = sign*(72*.0254-.262481), y = -sign*(72*.0254-.15835);
            double heading = sign > 0 ? Math.PI : 0;
            s.model.drive.setPose(x/.0254,y/.0254,heading);
            h.pinpoint.attachChassisAt(x*1000,y*1000,heading);
            s.run();
            int complete = s.trace.samples.stream().filter(v -> v.phase == OpModePhase.ACTIVE
                    && v.telemetry.contains("Field rehome refinement : CORNER ONLY: Limelight refinement timed out"))
                    .mapToInt(v -> v.tick).findFirst().orElseThrow(() -> new AssertionError(s.diagnostic()));
            // Tags are below the 55-deg camera's image from the corner; feed waits for the 3-s timeout.
            assertTrue(complete > 25+140,s.diagnostic());
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick < complete) {
                assertEquals(.5,v.gatePosition,s.diagnostic());
                assertFalse(v.feedingThroughOpenGate(),s.diagnostic());
            }
            assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE
                    && v.tick > complete && v.feedingThroughOpenGate()),s.diagnostic());
            CompetitionSimTest.assertStopped(h);
        }
    }

    @Test void realDiagnosticsInitializeAndKeepMotorsOffAtStop() throws Exception {
        for (String opMode : new String[]{"PinpointTestOpMode", "LimelightTagTestOpMode", "GoalStateTestOpMode",
                "AutoAimTestOpMode", "PedroDriveTestOpMode", "ShotCalibrationOpMode"}) {
            TeleOpScenario s = new TeleOpScenario("diagnostic-" + opMode, opMode,
                    BioBuzzSimHardware.forOpMode(opMode), .02, 160, 100, (p,t,a,b) -> {});
            s.run();
            assertFalse(s.result.endedBeforeStop(), s.diagnostic());
            assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
            if (opMode.equals("PinpointTestOpMode")) {
                assertTrue(s.trace.at(OpModePhase.ACTIVE,60).telemetry.contains("LOCAL PINPOINT"),s.diagnostic());
                assertFalse(s.trace.at(OpModePhase.ACTIVE,60).telemetry.contains("Camera frame"),s.diagnostic());
            }
            if (s.hardware.wheels[0] != null) for (org.horizon36596.simloop.fakehardware.FakeMotor wheel : s.hardware.wheels) assertEquals(0, wheel.getPower());
        }
    }

    @Test void invertedTagConventionNeverLocalizesOrFeeds() throws Exception {
        BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), false);
        TeleOpScenario s = new TeleOpScenario("inverted-tag-convention", "RedMecanumTeleOp", h,
                .02, 160, 80, (p,t,a,b) -> a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0);
        s.model.drive.setPose(-12.75, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
        h.pinpoint.attachChassisAt(-12.75*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
        s.run();
        assertTrue(h.camera.getLatestResult().isValid(), "Geometric detections exist");
        assertFalse(s.trace.at(OpModePhase.ACTIVE, 60).telemetry.contains("Field localization : LOCALIZED"),
                "An outward-reporting camera must fail the facing check after the physical conversion");
        assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 60).gatePosition);
    }

    @Test void physicalTagConventionAcquiresAndSurvivesCameraLoss() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            TeleOpScenario s = new TeleOpScenario(alliance + "-vision", alliance + "MecanumTeleOp",
                    h, .02, 160, 200, (p,t,a,b) -> {
                // Real acquisition, then camera loss: local odometry must retain the reference.
                if (p == OpModePhase.ACTIVE && t == 80) h.camera.stop();
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                b.back = p == OpModePhase.ACTIVE && t >= 1 && t < 4; // diagnostics page shows the aiming field pose
            });
            double x = alliance.equals("Red") ? -12.75 : 12.75;
            s.model.drive.setPose(x, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
            s.run();
            assertTrue(s.trace.samples.stream().anyMatch(v -> v.telemetry.contains("Field localization : LOCALIZED")), s.diagnostic());
            // Multi-tag fusion: field pose matches the plant, and a stationary robot's turret target
            // stays put while RT is held (no correction-driven jerks).
            java.util.regex.Matcher pose = java.util.regex.Pattern
                    .compile("Aiming field pose \\(m, deg\\) : (-?[\\d.]+), (-?[\\d.]+) \\| (-?[\\d.]+)")
                    .matcher(s.trace.at(OpModePhase.ACTIVE, 70).telemetry);
            assertTrue(pose.find(), s.diagnostic());
            assertEquals(x*.0254, Double.parseDouble(pose.group(1)), .03, s.diagnostic());
            assertEquals(BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*.0254, Double.parseDouble(pose.group(2)), .03, s.diagnostic());
            assertEquals(0, org.firstinspires.ftc.teamcode.RobotMath.wrap(Math.toRadians(Double.parseDouble(pose.group(3)))-Math.PI),
                    Math.toRadians(1.5), s.diagnostic());
            double low = Double.POSITIVE_INFINITY, high = Double.NEGATIVE_INFINITY;
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick >= 20 && v.tick < 80) {
                low = Math.min(low, v.yaw); high = Math.max(high, v.yaw);
            }
            assertTrue(high-low < .5, "Turret yaw range while stationary (deg): " + (high-low) + " " + s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 180).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 180).gatePosition, s.diagnostic());
            CompetitionSimTest.assertStopped(h);
        }
    }

    @Test void bothAlliancesSpinMoreThanOneTurnWhileAutoAimRespectsDeadZoneAndUnwinds() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            float turn = alliance.equals("Red") ? .35f : -.35f; // Opposite spin directions cover both wraps.
            TeleOpScenario s = new TeleOpScenario(alliance + "-spin-wrap", alliance + "MecanumTeleOp",
                    h, .02, 160, 560, (p,t,a,b) -> {
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                a.right_stick_x = p == OpModePhase.ACTIVE && t >= 40 && t < 440 ? turn : 0;
            });
            double x = alliance.equals("Red") ? -12.75 : 12.75;
            s.model.drive.setPose(x, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
            s.run();
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 30).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            double turned = 0, previous = Double.NaN, limitSide = 0;
            boolean unwound = false;
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
                if (!Double.isNaN(previous)) turned += org.firstinspires.ftc.teamcode.RobotMath.wrap(v.headingRad - previous);
                previous = v.headingRad;
                // Turret never enters the rear dead zone (measured, small servo overshoot allowed).
                assertTrue(Math.abs(v.yaw) <= TurretSubsystem.ROTATION_MAX_ANGLE_DEG + 1, "yaw " + v.yaw + " at " + v.tick + " " + s.diagnostic());
                if (Math.abs(v.yaw) > 150) limitSide = Math.signum(v.yaw);
                // After reaching one limit it swings back through home to the other side (the long way).
                if (limitSide != 0 && Math.signum(v.yaw) == -limitSide && Math.abs(v.yaw) > 5) unwound = true;
            }
            assertTrue(Math.abs(turned) > 2*Math.PI + .5, "robot turned only " + Math.toDegrees(turned) + " deg " + s.diagnostic());
            assertTrue(unwound, "turret never unwound past home after reaching a limit " + s.diagnostic());
            // After the spin stops, odometry still aims at the selected opening.
            TeleOpTrace.Sample end = s.trace.at(OpModePhase.ACTIVE, 550);
            assertTrue(end.telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            MotionSample truth = new MotionSample(0, end.xIn*.0254, end.yIn*.0254, end.headingRad, 0,0,0,0,.02,true,true,"plant");
            GoalTarget goal = TeleOpFieldReference.target(alliance.equals("Red") ? TurretSubsystem.Alliance.RED
                    : TurretSubsystem.Alliance.BLUE, TeleOpFieldReference.nearestSide(truth.y, TurretSubsystem.HiveSide.AUDIENCE));
            double expected = TurretSubsystem.reachableYawDegrees(
                    Math.toDegrees(TeleOpFieldReference.yawRadians(truth, goal, ShootingConfiguration.MOUNT)), end.yaw);
            assertEquals(expected, end.yaw, 2.0, s.diagnostic());
            CompetitionSimTest.assertStopped(h);
        }
    }

    /**
     * Steady spin below the synthetic 180 deg/s turret slew: away from the rear dead zone and the long-way
     * unwind, the measured turret yaw stays on the plant's true goal bearing every loop, not only after the
     * spin stops. Exercises same-loop servo writes and the aim lead; the sim turret has no servo lag, so this
     * cannot establish the robot's AIM_LEAD_SECONDS.
     */
    @Test void bothAlliancesTrackTheGoalWithinShotToleranceWhileSpinning() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            float turn = alliance.equals("Red") ? .3f : -.3f;
            TeleOpScenario s = new TeleOpScenario(alliance + "-spin-tracking", alliance + "MecanumTeleOp",
                    h, .02, 160, 420, (p,t,a,b) -> {
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                a.right_stick_x = p == OpModePhase.ACTIVE && t >= 40 && t < 400 ? turn : 0;
            });
            double x = alliance.equals("Red") ? -12.75 : 12.75;
            s.model.drive.setPose(x, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
            s.run();
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 30).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            TurretSubsystem.Alliance goalAlliance = alliance.equals("Red") ? TurretSubsystem.Alliance.RED : TurretSubsystem.Alliance.BLUE;
            double maxRate = 0, sumError = 0, maxError = 0, previous = Double.NaN;
            int tracking = 0, settled = 0, worstTick = -1;
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick >= 80 && v.tick < 400) {
                if (!Double.isNaN(previous)) maxRate = Math.max(maxRate,
                        Math.abs(Math.toDegrees(org.firstinspires.ftc.teamcode.RobotMath.wrap(v.headingRad - previous))) / .02);
                previous = v.headingRad;
                MotionSample truth = new MotionSample(0, v.xIn*.0254, v.yIn*.0254, v.headingRad, 0,0,0,0,.02,true,true,"plant");
                GoalTarget goal = TeleOpFieldReference.target(goalAlliance,
                        TeleOpFieldReference.nearestSide(truth.y, TurretSubsystem.HiveSide.AUDIENCE));
                double bearing = Math.toDegrees(TeleOpFieldReference.yawRadians(truth, goal, ShootingConfiguration.MOUNT));
                double error = Math.abs(TurretSubsystem.normalizeAngleDegrees(bearing - v.yaw));
                // The rear dead zone and the long-way unwind are excluded: the turret cannot point there.
                double edge = TurretSubsystem.ROTATION_MAX_ANGLE_DEG - 10;
                settled = Math.abs(bearing) < edge && Math.abs(v.yaw) < edge && error < 10 ? settled + 1 : 0;
                if (settled > 10) {
                    tracking++; sumError += error;
                    if (error > maxError) { maxError = error; worstTick = v.tick; }
                }
            }
            String stats = String.format(java.util.Locale.US, "%s: rate %.0f deg/s | tracking %d | mean %.2f | max %.2f deg at %d ",
                    alliance, maxRate, tracking, tracking == 0 ? Double.NaN : sumError / tracking, maxError, worstTick);
            System.out.println("Spin tracking " + stats);
            assertTrue(maxRate > 60 && maxRate < 170, "spin must stay below the synthetic turret slew " + stats + s.diagnostic());
            assertTrue(tracking > 100, "too few tracking samples " + stats + s.diagnostic());
            assertTrue(maxError <= org.firstinspires.ftc.teamcode.ShotReadiness.YAW_TOLERANCE_DEG, stats + s.diagnostic());
            assertTrue(sumError / tracking <= 1.0, stats + s.diagnostic());
            CompetitionSimTest.assertStopped(h);
        }
    }

    /**
     * Odometry-only auto aim while the chassis drives forward, back, right and left across the goal bearing (chassis
     * 45 deg from the line of sight, so both pods carry lateral motion). No camera: Pinpoint alone must keep the turret
     * on the plant's true bearing and turn it the same way the bearing turns. A pod direction that disagrees with the
     * physical pod mirrors that axis and turns the turret the wrong way (robot, 2026-11-13: X pod).
     */
    @Test void bothAlliancesTrackTheGoalWhileDrivingAndStrafingAcrossIt() throws Exception {
        String[] phases = {"forward", "back", "right", "left"};
        final int start = 40, ticks = 40;
        for (String alliance : new String[]{"Red", "Blue"}) {
            boolean red = alliance.equals("Red");
            double sign = red ? 1 : -1;
            double x = sign * CrossingHandoffRedTeleOp.X_METERS, y = sign * CrossingHandoffRedTeleOp.Y_METERS;
            double heading = CrossingHandoffRedTeleOp.HEADING_RAD + (red ? 0 : Math.PI);
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot(); // No camera: odometry alone keeps the aim.
            TeleOpScenario s = new TeleOpScenario(alliance + "-aim-crossing", "sim.CrossingHandoff" + alliance + "TeleOp",
                    h, .02, 160, start + 4 * ticks + 40, (p,t,a,b) -> {
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                int phase = p == OpModePhase.ACTIVE && t >= start ? (t - start) / ticks : -1;
                a.left_stick_y = phase == 0 ? -.4f : phase == 1 ? .4f : 0;
                a.left_stick_x = phase == 2 ? .4f : phase == 3 ? -.4f : 0;
            });
            s.model.drive.setPose(x / .0254, y / .0254, heading);
            h.pinpoint.attachChassisAt(x * 1000, y * 1000, heading);
            s.run();
            assertTrue(s.trace.at(OpModePhase.ACTIVE, start - 10).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            TurretSubsystem.Alliance goalAlliance = red ? TurretSubsystem.Alliance.RED : TurretSubsystem.Alliance.BLUE;
            double[] bearing = new double[start + 4 * ticks + 40], yaw = new double[bearing.length];
            double maxError = 0;
            int worstTick = -1;
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick < bearing.length) {
                MotionSample truth = new MotionSample(0, v.xIn*.0254, v.yIn*.0254, v.headingRad, 0,0,0,0,.02,true,true,"plant");
                GoalTarget goal = TeleOpFieldReference.target(goalAlliance, TeleOpFieldReference.nearestSide(truth.y,
                        red ? TurretSubsystem.HiveSide.AUDIENCE : TurretSubsystem.HiveSide.FAR));
                bearing[v.tick] = TurretSubsystem.reachableYawDegrees(
                        Math.toDegrees(TeleOpFieldReference.yawRadians(truth, goal, ShootingConfiguration.MOUNT)), v.yaw);
                yaw[v.tick] = v.yaw;
                double error = Math.abs(TurretSubsystem.normalizeAngleDegrees(bearing[v.tick] - v.yaw));
                if (v.tick >= start - 10 && error > maxError) { maxError = error; worstTick = v.tick; }
            }
            StringBuilder stats = new StringBuilder(alliance + ": max error " + String.format(java.util.Locale.US, "%.2f deg at %d", maxError, worstTick));
            for (int i = 0; i < phases.length; i++) {
                int a = start + i * ticks, b = a + ticks;
                double turned = yaw[b] - yaw[a], needed = bearing[b] - bearing[a];
                stats.append(String.format(java.util.Locale.US, " | %s bearing %+.1f turret %+.1f", phases[i], needed, turned));
            }
            System.out.println("Crossing tracking " + stats);
            for (int i = 0; i < phases.length; i++) {
                int a = start + i * ticks, b = a + ticks;
                double turned = yaw[b] - yaw[a], needed = bearing[b] - bearing[a];
                assertTrue(Math.abs(needed) >= 5, phases[i] + " must move across the bearing: " + stats + s.diagnostic());
                assertEquals(Math.signum(needed), Math.signum(turned), phases[i] + " turret turned the wrong way: " + stats + s.diagnostic());
                assertEquals(needed, turned, 2.0, phases[i] + " " + stats + s.diagnostic());
            }
            assertTrue(maxError <= org.firstinspires.ftc.teamcode.ShotReadiness.YAW_TOLERANCE_DEG, stats + s.diagnostic());
            assertTrue(Math.abs(TurretSubsystem.normalizeAngleDegrees(bearing[bearing.length - 1] - yaw[yaw.length - 1])) <= 1.0,
                    stats + s.diagnostic());
            CompetitionSimTest.assertStopped(h);
        }
    }
}
