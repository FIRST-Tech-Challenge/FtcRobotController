package org.firstinspires.ftc.teamcode.sim;

import org.junit.jupiter.api.Test;
import org.horizon36596.simloop.opmode.OpModePhase;
import static org.junit.jupiter.api.Assertions.*;

class CompetitionSimTest {
    @Test void bothAlliancesLoadWhileAimingPauseForFireAndReportIndependentLights() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withLights();
            TeleOpScenario s = new TeleOpScenario(alliance + "-aim-collection-lights", alliance + "MecanumTeleOp",
                    h, .02, 160, 170, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_trigger = active ? 1 : 0;
                a.left_trigger = active && t >= 40 && t < 65 ? 1 : 0;
                b.right_trigger = active && t >= 80 && t < 120 ? 1 : 0;
                b.left_trigger = active && t >= 105 && t < 120 ? 1 : 0;
            });
            s.observer = v -> {
                if (v.phase == OpModePhase.ACTIVE && v.tick == 20) {
                    assertEquals(1700, h.ledIntake.getPulseWidth(), 1);
                    assertEquals(1300, h.ledShooter.getPulseWidth(), 1);
                    assertEquals(1200, h.ledStatus.getPulseWidth(), 1); // Homed, no field reference.
                }
                if (v.phase == OpModePhase.ACTIVE && v.tick == 110)
                    assertEquals(1100, h.ledShooter.getPulseWidth(), 1);
            };
            s.run();
            for (int tick : new int[]{20, 70, 125}) {
                assertEquals(1, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
                // Loading while aiming runs stage 1 at full open-loop power.
                MechanismsSimTest.assertLoading(s.trace.at(OpModePhase.ACTIVE, tick), s.diagnostic());
                assertEquals(1, s.trace.at(OpModePhase.ACTIVE, tick).transfer2Power, s.diagnostic()); // Linked to stage 1.
                assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, tick).gatePosition, s.diagnostic());
            }
            for (int tick : new int[]{50, 85, 100, 110})
                assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 50).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 100).transfer2Power, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 100).telemetry.contains("Shot : MANUAL FEED"));
            assertEquals(600, h.ledIntake.getPulseWidth(), 1);
            assertEquals(600, h.ledShooter.getPulseWidth(), 1);
            assertEquals(600, h.ledStatus.getPulseWidth(), 1);
            assertStopped(h);
        }
    }

    @Test void bothAlliancesApplyAndClearAutoTrimsWithoutChangingManualDefaults() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            TeleOpScenario s = new TeleOpScenario(alliance + "-SYNTHETIC-trims", "sim.Synthetic" + alliance + "TeleOp",
                    h, .02, 160, 190, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_trigger = active && t < 160 ? 1 : 0;
                b.dpad_up = b.y = active && t >= 50 && t < 75;
                b.right_stick_x = active && t >= 50 && t < 60 ? .5f : 0;
                b.dpad_right = active && t >= 110 && t < 135;
            });
            double x = (alliance.equals("Red") ? -12.75 : 12.75) + 8;
            s.model.drive.setPose(x, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
            s.run();
            TeleOpTrace.Sample before = s.trace.at(OpModePhase.ACTIVE, 40), trim = s.trace.at(OpModePhase.ACTIVE, 95);
            assertEquals(org.firstinspires.ftc.teamcode.TurretSubsystem.pitchDegreesToServoPos(5.5), trim.hood, 1e-9);
            assertEquals(2850.0 / 60 * 28, trim.flywheelVelocity, 50.0 / 60 * 28, s.diagnostic());
            assertTrue(trim.telemetry.contains("-1.5 / 0.5 / 50"), s.diagnostic());
            assertEquals(before.hood, s.trace.at(OpModePhase.ACTIVE, 145).hood, 1e-9);
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 145).telemetry.contains("0.0 / 0.0 / 0"));
            assertTrue(java.util.regex.Pattern.compile("Flywheel \\(RPM\\) : -?\\d+ / 3000 \\|")
                    .matcher(s.trace.at(OpModePhase.ACTIVE, 185).telemetry).find(), s.diagnostic());
            for (TeleOpTrace.Sample v : s.trace.samples) assertEquals(.5, v.gatePosition, s.diagnostic());
            assertStopped(h);
        }
    }
    /**
     * The installed competition profile is uncalibrated, so auto aim sets yaw only and the shot uses the manual
     * hood/RPM. While RT is held, GP2 D-pad Up / Y must change those (previously they changed hidden trims).
     */
    @Test void bothAlliancesAdjustManualHoodAndRpmWhileAimingWithoutACalibratedProfile() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-uncalibrated-aim-manual-hood-rpm", alliance + "MecanumTeleOp",
                    h, .02, 160, 120, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_trigger = active ? 1 : 0;
                b.dpad_up = b.y = active && t >= 50 && t < 75; // One press of each, held 0.5 s.
            });
            s.run();
            TeleOpTrace.Sample before = s.trace.at(OpModePhase.ACTIVE, 40), after = s.trace.at(OpModePhase.ACTIVE, 95);
            assertEquals(org.firstinspires.ftc.teamcode.TurretSubsystem.pitchDegreesToServoPos(0), before.hood, 1e-9, s.diagnostic());
            assertEquals(org.firstinspires.ftc.teamcode.TurretSubsystem.pitchDegreesToServoPos(.5), after.hood, 1e-9, s.diagnostic());
            assertTrue(java.util.regex.Pattern.compile("Flywheel \\(RPM\\) : -?\\d+ / 3050 \\|")
                    .matcher(after.telemetry).find(), s.diagnostic());
            assertTrue(after.telemetry.contains("Auto trims (yaw deg / hood deg / RPM) : 0.0 / 0.0 / 0"), s.diagnostic());
            assertStopped(h);
        }
    }
    /**
     * Autonomous left a same-alliance field pose: TeleOp applies it in INIT once the Pinpoint settles and auto aim
     * points at the goal from the first ACTIVE loops, with no camera and no corner rehome.
     */
    @Test void bothAlliancesStartTeleOpAimingFromTheHandedOffAutonomousPose() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            boolean red = alliance.equals("Red");
            double sign = red ? 1 : -1;
            double x = sign * HandoffRedTeleOp.X_METERS, y = sign * HandoffRedTeleOp.Y_METERS;
            double heading = HandoffRedTeleOp.HEADING_RAD + (red ? 0 : Math.PI);
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot(); // No camera: only the handoff can localize.
            TeleOpScenario s = new TeleOpScenario(alliance + "-auto-pose-handoff", "sim.Handoff" + alliance + "TeleOp",
                    h, .02, 160, 100, (p,t,a,b) -> a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0);
            s.model.drive.setPose(x / .0254, y / .0254, heading);
            h.pinpoint.attachChassisAt(x * 1000, y * 1000, heading);
            s.run();
            TeleOpTrace.Sample early = s.trace.at(OpModePhase.INIT_LOOP, 5), init = s.trace.at(OpModePhase.INIT_LOOP, 150);
            assertTrue(early.telemetry.contains("| LOCAL"), s.diagnostic()); // Pinpoint still settling.
            assertTrue(init.telemetry.contains("Auto pose handoff : APPLIED from autonomous"), s.diagnostic());
            assertTrue(init.telemetry.contains("| EXPLICIT FIELD"), s.diagnostic());
            TeleOpTrace.Sample end = s.trace.at(OpModePhase.ACTIVE, 90);
            assertTrue(end.telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            org.firstinspires.ftc.teamcode.MotionSample truth = new org.firstinspires.ftc.teamcode.MotionSample(0,
                    end.xIn * .0254, end.yIn * .0254, end.headingRad, 0, 0, 0, 0, .02, true, true, "plant");
            org.firstinspires.ftc.teamcode.GoalTarget goal = org.firstinspires.ftc.teamcode.TeleOpFieldReference.target(
                    red ? org.firstinspires.ftc.teamcode.TurretSubsystem.Alliance.RED : org.firstinspires.ftc.teamcode.TurretSubsystem.Alliance.BLUE,
                    org.firstinspires.ftc.teamcode.TeleOpFieldReference.nearestSide(truth.y,
                            org.firstinspires.ftc.teamcode.TurretSubsystem.HiveSide.AUDIENCE));
            double expected = org.firstinspires.ftc.teamcode.TurretSubsystem.reachableYawDegrees(Math.toDegrees(
                    org.firstinspires.ftc.teamcode.TeleOpFieldReference.yawRadians(truth, goal,
                            org.firstinspires.ftc.teamcode.ShootingConfiguration.MOUNT)), end.yaw);
            assertEquals(expected, end.yaw, 2.0, s.diagnostic());
            assertStopped(h);
        }
    }
    /**
     * The handed-off pose is 0.6 m and 12 deg wrong (robot moved after autonomous). The camera sees the raised
     * audience-cell tags during INIT, confirms the large disagreement, re-localizes, and auto aim then points at
     * the real goal. Before this change such a disagreement was ignored and the turret aimed from the wrong pose.
     */
    @Test void bothAlliancesCameraReLocalizesAWrongHandedOffPose() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            boolean red = alliance.equals("Red");
            double x = (red ? 1 : -1) * MisplacedHandoffRedTeleOp.TRUE_X_METERS, y = MisplacedHandoffRedTeleOp.TRUE_Y_METERS;
            double heading = MisplacedHandoffRedTeleOp.TRUE_HEADING_RAD;
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            TeleOpScenario s = new TeleOpScenario(alliance + "-wrong-handoff-relocalized", "sim.MisplacedHandoff" + alliance + "TeleOp",
                    h, .02, 250, 100, (p,t,a,b) -> a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0);
            s.model.drive.setPose(x / .0254, y / .0254, heading);
            h.pinpoint.attachChassisAt(x * 1000, y * 1000, heading);
            s.run();
            int disagree = -1, relocalized = -1;
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.INIT_LOOP) {
                if (disagree < 0 && v.telemetry.contains("Camera pose check : CAMERA DISAGREES")) disagree = v.tick;
                if (relocalized < 0 && v.telemetry.contains("Camera pose check : RE-LOCALIZED from camera")) relocalized = v.tick;
            }
            assertTrue(s.trace.at(OpModePhase.INIT_LOOP, 60).telemetry.contains("Auto pose handoff : APPLIED"), s.diagnostic());
            assertTrue(disagree >= 0, "never reported the disagreement " + s.diagnostic());
            assertTrue(relocalized > disagree, "disagree " + disagree + ", re-localized " + relocalized + " " + s.diagnostic());
            TeleOpTrace.Sample end = s.trace.at(OpModePhase.ACTIVE, 90);
            assertTrue(end.telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            org.firstinspires.ftc.teamcode.TurretSubsystem.Alliance goalAlliance = red
                    ? org.firstinspires.ftc.teamcode.TurretSubsystem.Alliance.RED : org.firstinspires.ftc.teamcode.TurretSubsystem.Alliance.BLUE;
            org.firstinspires.ftc.teamcode.MotionSample truth = new org.firstinspires.ftc.teamcode.MotionSample(0,
                    end.xIn * .0254, end.yIn * .0254, end.headingRad, 0, 0, 0, 0, .02, true, true, "plant");
            org.firstinspires.ftc.teamcode.GoalTarget goal = org.firstinspires.ftc.teamcode.TeleOpFieldReference.target(goalAlliance,
                    org.firstinspires.ftc.teamcode.TeleOpFieldReference.nearestSide(truth.y,
                            org.firstinspires.ftc.teamcode.TurretSubsystem.HiveSide.AUDIENCE));
            double expected = org.firstinspires.ftc.teamcode.TurretSubsystem.reachableYawDegrees(Math.toDegrees(
                    org.firstinspires.ftc.teamcode.TeleOpFieldReference.yawRadians(truth, goal,
                            org.firstinspires.ftc.teamcode.ShootingConfiguration.MOUNT)), end.yaw);
            // Aiming from the wrong handed-off pose would have been far off: the scenario really tests the fix.
            double errorX = (red ? 1 : -1) * MisplacedHandoffRedTeleOp.ERROR_X_METERS;
            double errorHeading = (red ? 1 : -1) * MisplacedHandoffRedTeleOp.ERROR_HEADING_RAD;
            org.firstinspires.ftc.teamcode.MotionSample wrong = new org.firstinspires.ftc.teamcode.MotionSample(0,
                    x + errorX, y, heading + errorHeading, 0, 0, 0, 0, .02, true, true, "handoff");
            double wrongYaw = Math.toDegrees(org.firstinspires.ftc.teamcode.TeleOpFieldReference.yawRadians(wrong, goal,
                    org.firstinspires.ftc.teamcode.ShootingConfiguration.MOUNT));
            assertTrue(Math.abs(org.firstinspires.ftc.teamcode.TurretSubsystem.normalizeAngleDegrees(wrongYaw - expected)) > 20,
                    "wrong-pose aim " + wrongYaw + " vs " + expected);
            assertEquals(expected, end.yaw, 2.0, s.diagnostic());
            System.out.println("Wrong handoff " + alliance + ": disagreement at INIT tick " + disagree + ", re-localized at "
                    + relocalized + ", final yaw " + end.yaw + " vs " + expected + " (wrong pose would aim " + wrongYaw + ")");
            assertStopped(h);
        }
    }
    @Test void bothAlliancesFieldRehomeInInitAndActiveUsesPressEdgesWithoutCamera() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (boolean inInit : new boolean[]{false, true}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-field-rehome-" + inInit,
                    alliance + "MecanumTeleOp", h, .02, 160, 130, (p,t,a,b) -> {
                b.dpad_left = p == OpModePhase.INIT_LOOP ? inInit && t >= 150
                        : p == OpModePhase.ACTIVE && ((t >= (inInit ? 0 : 20) && t < 60)
                        || (t >= 80 && t < 110));
                a.left_stick_y = p == OpModePhase.ACTIVE ? -.25f : 0;
                // Rehoming remains available while the driver requests automatic aiming.
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
            });
            java.util.Map<Integer, double[]> poses = new java.util.HashMap<>();
            s.observer = v -> {
                if (v.phase == OpModePhase.ACTIVE) {
                    org.firstinspires.ftc.robotcore.external.navigation.Pose2D p = h.pinpoint.getPosition();
                    poses.put(v.tick, new double[]{
                            p.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER),
                            p.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER),
                            p.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)});
                }
            };
            s.run();
            double sign = alliance.equals("Blue") ? 1 : -1;
            // Independent placement calculation: 144-in field, rear 262.481 mm, side 158.35 mm.
            double expectedX = sign * (72 * .0254 - .262481);
            double expectedY = -sign * (72 * .0254 - .15835);
            double expectedHeading = alliance.equals("Blue") ? Math.PI : 0;
            int firstPress = inInit ? 0 : 20;
            if (!inInit) assertFalse(s.trace.at(OpModePhase.ACTIVE, 10).telemetry.contains("Field localization : LOCALIZED"));
            for (int tick : new int[]{firstPress, 80}) {
                assertEquals(expectedX, poses.get(tick)[0], inInit && tick == 0 ? .002 : 1e-6, s.diagnostic());
                assertEquals(expectedY, poses.get(tick)[1], 1e-6, s.diagnostic());
                assertEquals(0, org.firstinspires.ftc.teamcode.RobotMath.wrap(poses.get(tick)[2] - expectedHeading), 1e-6);
                assertTrue(s.trace.at(OpModePhase.ACTIVE, tick).telemetry.contains("| EXPLICIT FIELD"), s.diagnostic());
                // A reset deliberately invalidates the first acceleration sample; the next loop must localize.
                if (!(inInit && tick == 0)) {
                    assertFalse(s.trace.at(OpModePhase.ACTIVE, tick).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
                }
                assertTrue(s.trace.at(OpModePhase.ACTIVE, tick + 1).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            }
            // Holding through START or for many ACTIVE ticks must allow odometry movement, not pin the pose.
            assertTrue(sign * (poses.get(50)[0] - poses.get(firstPress)[0]) < -.01, s.diagnostic());
            assertTrue(sign * (poses.get(105)[0] - poses.get(80)[0]) < -.01, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 50).powers[0] != 0, s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 105).gatePosition, s.diagnostic());
            assertStopped(h);
        }
    }

    /**
     * A corner rehome pressed while TeleOp INIT is still recalibrating the Pinpoint IMU waits for a settled READY
     * device (the rule the autonomous handoff and Pedro setFieldPose already follow), then applies exactly once.
     * The SimLoop firmware keeps a pose written during calibration, so this checks the ordering, not pose loss.
     */
    @Test void bothAlliancesCornerRehomeDuringInitCalibrationWaitsForSettledPinpoint() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-corner-rehome-during-calibration",
                    alliance + "MecanumTeleOp", h, .02, 100, 20, (p,t,a,b) -> {
                b.dpad_left = p == OpModePhase.INIT_LOOP && t < 3; // Pressed before the first loop's Pinpoint read.
            });
            java.util.Map<Integer, double[]> poses = new java.util.HashMap<>();
            s.observer = v -> {
                if (v.phase != OpModePhase.INIT_LOOP) return;
                org.firstinspires.ftc.robotcore.external.navigation.Pose2D p = h.pinpoint.getPosition();
                poses.put(v.tick, new double[]{
                        p.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER),
                        p.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER),
                        p.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)});
            };
            s.run();
            double sign = alliance.equals("Blue") ? 1 : -1;
            double cornerX = sign * (72 * .0254 - .262481), cornerY = -sign * (72 * .0254 - .15835);
            double cornerHeading = alliance.equals("Blue") ? Math.PI : 0;
            // Calibration (0.25 s) plus the 0.5 s READY settle: nothing is written for the first ~0.7 s.
            for (int tick = 0; tick <= 30; tick++) {
                assertEquals(0, poses.get(tick)[0], 1e-6, "tick " + tick + " " + s.diagnostic());
                assertEquals(0, poses.get(tick)[1], 1e-6, "tick " + tick + " " + s.diagnostic());
            }
            assertTrue(s.trace.at(OpModePhase.INIT_LOOP, 5).telemetry.contains("Corner rehome (GP2 D-pad Left) : WAITING"),
                    s.diagnostic());
            int applied = -1;
            for (int tick = 0; tick < 100 && applied < 0; tick++)
                if (Math.abs(poses.get(tick)[0] - cornerX) < 1e-6) applied = tick;
            assertTrue(applied > 30 && applied < 60, "applied at INIT tick " + applied + " " + s.diagnostic());
            for (int tick = applied; tick < 100; tick++) {
                assertEquals(cornerX, poses.get(tick)[0], 1e-6, s.diagnostic());
                assertEquals(cornerY, poses.get(tick)[1], 1e-6, s.diagnostic());
                assertEquals(0, org.firstinspires.ftc.teamcode.RobotMath.wrap(poses.get(tick)[2] - cornerHeading), 1e-6);
            }
            assertTrue(s.trace.at(OpModePhase.INIT_LOOP, 99).telemetry.contains("Corner rehome (GP2 D-pad Left) : APPLIED"),
                    s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 10).telemetry.contains("Field localization : LOCALIZED"), s.diagnostic());
            assertFalse(s.trace.at(OpModePhase.ACTIVE, 10).telemetry.contains("Corner rehome"), s.diagnostic());
            System.out.println("Corner rehome during calibration " + alliance + ": applied at INIT tick " + applied);
            assertStopped(h);
        }
    }

    @Test void bothAlliancesFeedContinuouslyOnEitherManualControlAndIgnoreGp2B() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (int feedPad : new int[]{1, 2}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-continuous-feed-gp" + feedPad,
                    alliance + "MecanumTeleOp", h, .02, 160, 300, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                boolean feed = active && t >= 10 && (t < 246 || t >= 260);
                a.a = feed && feedPad == 1;
                a.left_trigger = active && t < 10 ? 1 : 0; // LT alone no longer requests manual feeding.
                b.right_trigger = feed && feedPad == 2 ? 1 : 0;
                b.b = active && ((t >= 60 && t < 130) || (t >= 150 && t < 200));
                b.left_trigger = active && t >= 230 && t < 246 ? 1 : 0;
                a.left_stick_y = active ? -.2f : 0;
            });
            s.run();
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 5).gatePosition, s.diagnostic());
            assertFalse(s.trace.at(OpModePhase.ACTIVE, 5).feedingThroughOpenGate(), s.diagnostic());
            // Continuous only: no staggered gate delay, timed window or recovery pause. The feed runs
            // unbroken while held, including across GP2 B presses (GP2 B no longer selects a feed mode).
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick >= 10 && v.tick < 230) {
                assertEquals(1, v.transfer1Power, s.diagnostic());
                assertEquals(1, v.transfer2Power, s.diagnostic());
                assertTrue(v.feedingThroughOpenGate(), s.diagnostic());
                assertFalse(v.telemetry.contains("STAGGERED"), s.diagnostic());
            }
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 280).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 280).transfer2Power, s.diagnostic());
            // Inhibit and release still stop the feed.
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 235).gatePosition, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 235).transfer1Power, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 235).transfer2Power, s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 250).gatePosition, s.diagnostic());
            assertFalse(s.trace.at(OpModePhase.ACTIVE, 250).feedingThroughOpenGate(), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 220).xIn
                    > s.trace.at(OpModePhase.ACTIVE, 60).xIn, s.diagnostic());
            assertStopped(h);
        }
    }

    @Test void syntheticMeasuredProfileFeedsThenOdometryFaultBlocksFurtherShots() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withCamera(BioBuzzSimHardware.raisedAudienceTags(), true);
            TeleOpScenario s = new TeleOpScenario(alliance + "-SYNTHETIC-calibration", "sim.Synthetic" + alliance + "TeleOp",
                    h, .02, 160, 480, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_trigger = active && (t < 230 || t >= 260) ? 1 : 0;
                a.left_trigger = active && t >= 80 && (t < 140 || t >= 170) ? 1 : 0;
                if (active && t == 320)
                    h.pinpoint.forceFault(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.NOT_READY);
            });
            // Start off the hive centerline so preparation must move yaw as well as hood/RPM.
            double x = (alliance.equals("Red") ? -12.75 : 12.75) + 8;
            s.model.drive.setPose(x, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
            h.pinpoint.attachChassisAt(x*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
            s.run();
            // With valid synthetic calibration, RT prepares all three axes without opening the gate.
            TeleOpTrace.Sample prepared = s.trace.at(OpModePhase.ACTIVE, 75);
            assertEquals(org.firstinspires.ftc.teamcode.TurretSubsystem.pitchDegreesToServoPos(5), prepared.hood, 1e-9);
            assertEquals(2800.0 / 60 * 28, prepared.flywheelVelocity, 100.0 / 60 * 28, s.diagnostic());
            assertTrue(Math.abs(prepared.yaw) > 3, s.diagnostic());
            // Releasing either trigger stops automatic feeding; RT alone continues preparation.
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE
                    && (v.tick < 80 || (v.tick >= 140 && v.tick < 170) || (v.tick >= 230 && v.tick < 260))) {
                assertFalse(v.feedingThroughOpenGate(), s.diagnostic());
                assertEquals(.5, v.gatePosition, s.diagnostic());
            }
            for (int start : new int[]{80, 170, 260}) {
                assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE
                        && v.tick >= start && v.tick < start + 60
                        && v.feedingThroughOpenGate()), s.diagnostic());
            }
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick > 420) {
                assertFalse(v.feedingThroughOpenGate(), s.diagnostic()); assertEquals(.5, v.gatePosition, s.diagnostic());
            }
            assertStopped(h);
        }
    }

    @Test void missingRequiredTransferHardwareCleansUpInitializedOutputs() throws Exception {
        BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
        h.map.remove(null, "transfer_motor_2", h.transfer2);
        TeleOpScenario s = new TeleOpScenario("partial-init", "RedMecanumTeleOp", h,
                .02, 10, 10, (p,t,a,b) -> {});
        assertThrows(RuntimeException.class, s::run);
        assertStopped(h);
    }

    @Test void bothAlliancesManualDriveQueueFeedAndInhibit() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (int feedPad : new int[]{1, 2}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot().withLights();
            TeleOpScenario s = new TeleOpScenario(alliance + "-manual-gp" + feedPad, alliance + "MecanumTeleOp",
                    h, .02, 160, 350, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.left_stick_y = active && t < 40 ? -.5f : 0;
                a.left_bumper = active && ((t >= 40 && t < 50) || (t >= 60 && t < 70)
                        || (t >= 155 && t < 160) || (t >= 220 && t < 230));
                a.right_bumper = active && ((t >= 60 && t < 70) || (t >= 145 && t < 155)
                        || (t >= 160 && t < 175) || (t >= 200 && t < 210));
                b.right_bumper = active && ((t >= 90 && t < 100) || (t >= 155 && t < 160)
                        || t >= 260);
                b.left_bumper = active && ((t >= 105 && t < 115) || (t >= 140 && t < 145));
                h.distance.setSimulatedInputState(active && t < 140);
                boolean manualFeed = active && t >= 160 && t < 290;
                a.a = feedPad == 1 && manualFeed;
                b.right_trigger = feedPad == 2 && manualFeed ? 1 : 0;
                b.left_trigger = active && t >= 260 ? 1 : 0;
                b.right_stick_x = active && t < 40 ? .25f : 0;
                b.dpad_up = active && t < 40;
                a.right_stick_button = active && t == 50;
            });
            // Actual PWM pulse widths the TeleOp writes to the intake and shooter lights each ACTIVE loop.
            java.util.Map<Integer, Double> intakeLight = new java.util.HashMap<>();
            java.util.Map<Integer, Double> shooterLight = new java.util.HashMap<>();
            s.observer = v -> {
                if (v.phase != OpModePhase.ACTIVE) return;
                intakeLight.put(v.tick, (double) h.ledIntake.getPulseWidth());
                shooterLight.put(v.tick, (double) h.ledShooter.getPulseWidth());
            };
            s.run();
            assertFalse(s.result.endedBeforeStop(), s.diagnostic());
            assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 35).xIn > 0, s.diagnostic());
            // 0.5 s HIGH (about tick 25) confirms full: the intake stops (no reversal) while stage 1
            // runs on 0.5 s to seat the fourth ball (GP1 LB at tick 40 pauses loading and ends it).
            MechanismsSimTest.assertLoading(s.trace.at(OpModePhase.ACTIVE, 35), s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 35).transfer2Power, s.diagnostic()); // Run-on on both motors.
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 35).intakePower, s.diagnostic());
            // Intake light follows collection: blue loading, green full, orange LB reverse, off for
            // both bumpers, blue for the GP2 RB override, green again once released while full.
            assertEquals(1700, intakeLight.get(10), 1, s.diagnostic());
            assertEquals(1500, intakeLight.get(35), 1, s.diagnostic());
            assertEquals(1200, intakeLight.get(45), 1, s.diagnostic());
            assertEquals(600, intakeLight.get(65), 1, s.diagnostic());
            assertEquals(1700, intakeLight.get(95), 1, s.diagnostic());
            assertEquals(1500, intakeLight.get(130), 1, s.diagnostic());
            // Shooter light: red while GP2 LT inhibits the held manual feed.
            assertEquals(1100, shooterLight.get(270), 1, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 55).transfer1Power, s.diagnostic());
            assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 45).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 65).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 130).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 130).transfer1Power, s.diagnostic());
            // RB overrides confirmed full on GP2 as well as GP1 (both linked transfer motors), without opening the gate.
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 95).intakePower, s.diagnostic());
            MechanismsSimTest.assertLoading(s.trace.at(OpModePhase.ACTIVE, 95), s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 95).transfer2Power, s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 95).gatePosition, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 102).transfer1Power, s.diagnostic());
            // Reverse only powers intake, even with space; conflicting controls across pads stop collection.
            for (int tick : new int[]{110, 142, 157}) {
                assertEquals(tick == 157 ? 0 : -1, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
                assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).transfer1Power, s.diagnostic());
                assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).transfer2Power, s.diagnostic());
                assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, tick).gatePosition, s.diagnostic());
            }
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 150).intakePower, s.diagnostic());
            MechanismsSimTest.assertLoading(s.trace.at(OpModePhase.ACTIVE, 150), s.diagnostic());
            // Manual shot feed (from tick 160) also runs stage 1 at full open-loop power.
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 200).transfer1Power, s.diagnostic());
            assertFalse(s.trace.at(OpModePhase.ACTIVE, 200).transfer1VelocityMode, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 200).transfer2Power, s.diagnostic());
            assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 225).intakePower, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 225).transfer1Power, s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 225).transfer2Power, s.diagnostic());
            assertTrue(s.trace.samples.stream().anyMatch(TeleOpTrace.Sample::feedingThroughOpenGate), s.diagnostic());
            assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 270).gatePosition, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 270).transfer1Power, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 270).transfer2Power, s.diagnostic());
            // Intake is independent, but forward override cannot defeat the priority transfer inhibit.
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 270).intakePower, s.diagnostic());
            double openedAt = Double.NaN;
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
                if (Math.abs(v.gatePosition - .2) < 1e-9) {
                    if (Double.isNaN(openedAt)) openedAt = v.seconds;
                    // Default continuous mode starts both transfers on the opening command.
                    assertEquals(1, v.transfer1Power, s.diagnostic());
                    assertEquals(1, v.transfer2Power, s.diagnostic());
                } else openedAt = Double.NaN;
                if (v.tick >= 160 && v.tick < 170) assertEquals(1, v.transfer1Power, s.diagnostic());
            }
            assertEquals(165.0/355, s.trace.at(OpModePhase.ACTIVE, 55).depollinator, 1e-9);
            assertStopped(h);
        }
    }

    @Test void unknownFieldReferenceInhibitsAutoFireWithoutBlockingDrive() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            TeleOpScenario s = new TeleOpScenario(alliance + "-unlocalized", alliance + "MecanumTeleOp",
                    BioBuzzSimHardware.fullRobot(), .01, 2, 400, (p,t,a,b) -> {
                a.right_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                a.left_trigger = p == OpModePhase.ACTIVE ? 1 : 0;
                a.left_stick_y = p == OpModePhase.ACTIVE ? -.2f : 0;
            });
            s.run();
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 100).xIn > 0);
            for (TeleOpTrace.Sample v : s.trace.samples) {
                assertEquals(.5, v.gatePosition, s.diagnostic());
                assertFalse(v.feedingThroughOpenGate(), s.diagnostic());
            }
            assertStopped(s.hardware);
        }
    }

    @Test void bothAlliancesThreeMinuteEnduranceAndStopBeforeStart() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) for (int ticks : new int[]{0, 9000}) {
            TeleOpScenario s = new TeleOpScenario(alliance + "-duration-" + ticks, alliance + "MecanumTeleOp",
                    BioBuzzSimHardware.fullRobot(), .02, 160, ticks, (p,t,a,b) -> {});
            s.run(); assertStopped(s.hardware);
            assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
            if (ticks > 0) assertTrue(s.trace.at(OpModePhase.ACTIVE, 8999).flywheelVelocity > 1000);
        }
    }

    /**
     * Auto aim no longer caps chassis turning: full right stick reaches the same measured turn rate with and
     * without RT held, in both directions. Synthetic drive response only: not the robot's loaded spin rate.
     */
    @Test void bothAlliancesTurnAtFullRateWhileAutoAiming() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            float turn = alliance.equals("Red") ? 1f : -1f;
            TeleOpScenario s = new TeleOpScenario(alliance + "-aim-spin-uncapped", alliance + "MecanumTeleOp",
                    BioBuzzSimHardware.fullRobot(), .02, 160, 300, (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_stick_x = active && (t >= 20 && t < 120 || t >= 160 && t < 280) ? turn : 0;
                a.right_trigger = active && t >= 160 ? 1 : 0;
            });
            s.run();
            double free = peakTurnRateDegPerSecond(s, 60, 120), aiming = peakTurnRateDegPerSecond(s, 200, 280);
            String stats = String.format(java.util.Locale.US, "%s: free %.0f | aiming %.0f deg/s ",
                    alliance, free, aiming);
            System.out.println("Aim turn rate " + stats);
            assertTrue(free > 300, "synthetic full stick must spin fast " + stats + s.diagnostic());
            assertTrue(aiming >= free * .95, "auto aim must not cap turning " + stats + s.diagnostic());
            assertStopped(s.hardware);
        }
    }

    /** Five-tick (0.1 s) average plant heading rate, so one integration step cannot dominate. */
    static double peakTurnRateDegPerSecond(TeleOpScenario s, int fromTick, int toTick) {
        double peak = 0;
        for (int t = fromTick; t + 5 <= toTick; t++) {
            double a = s.trace.at(OpModePhase.ACTIVE, t).headingRad, b = s.trace.at(OpModePhase.ACTIVE, t + 5).headingRad;
            peak = Math.max(peak, Math.abs(Math.toDegrees(org.firstinspires.ftc.teamcode.RobotMath.wrap(b - a))) / (5 * .02));
        }
        return peak;
    }

    @Test void bothAlliancesToggleSlideOnGp2() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-slide", alliance + "MecanumTeleOp", h,
                    .02, 3, 340, (p,t,a,b) -> {
                b.right_stick_button = p == OpModePhase.ACTIVE && t >= 10 && t < 13;   // extend
                b.left_stick_button = p == OpModePhase.ACTIVE && t >= 160 && t < 163;  // home
            });
            s.run();
            assertTrue(s.trace.samples.stream().filter(v -> v.phase == OpModePhase.INIT || v.phase == OpModePhase.INIT_LOOP).allMatch(v -> v.slidePower == 0), s.diagnostic());
            TeleOpTrace.Sample up = s.trace.at(OpModePhase.ACTIVE, 150);
            assertEquals(700, up.slideTravelDeg, 3, s.diagnostic());
            assertEquals(0, up.slidePower, 0, s.diagnostic());
            assertTrue(up.telemetry.contains("Slide (GP2 RS/LS) : UP"), up.telemetry);
            TeleOpTrace.Sample down = s.trace.at(OpModePhase.ACTIVE, 330);
            assertEquals(0, down.slideTravelDeg, 3, s.diagnostic());
            assertEquals(0, h.slideServo.getPower(), 0);
            assertStopped(h);
        }
    }

    /** Wiring only: the flywheel fixture has synthetic response and no friction or ball load. */
    @Test void bothAlliancesRegulateTheFlywheelWithBangBangFusion() throws Exception {
        for (String alliance : new String[]{"Red", "Blue"}) {
            BioBuzzSimHardware h = BioBuzzSimHardware.fullRobot();
            TeleOpScenario s = new TeleOpScenario(alliance + "-fusion-flywheel", alliance + "MecanumTeleOp",
                    h, .02, 20, 300, (p,t,a,b) -> {});
            java.util.List<Double> activePower = new java.util.ArrayList<>();
            java.util.List<com.qualcomm.robotcore.hardware.DcMotor.RunMode> activeMode = new java.util.ArrayList<>();
            s.observer = v -> { if (v.phase == OpModePhase.ACTIVE) {
                activePower.add(h.flywheel.getPower()); activeMode.add(h.flywheel.getMode()); } };
            s.run();
            assertFalse(s.result.endedBeforeStop(), s.diagnostic());
            assertEquals(org.firstinspires.ftc.teamcode.TurretSubsystem.ShooterController.BANG_BANG_FUSION,
                    org.firstinspires.ftc.teamcode.MecanumTeleOp.FLYWHEEL_CONTROLLER);
            // Open-loop power from the main loop, not the hub velocity PIDF loop.
            for (com.qualcomm.robotcore.hardware.DcMotor.RunMode mode : activeMode)
                assertEquals(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER, mode, s.diagnostic());
            // From rest the error is far above 80 ticks/s: full power; overspeed only coasts.
            assertEquals(1, activePower.get(1), 1e-9, s.diagnostic());
            for (double power : activePower) assertTrue(power >= 0 && power <= 1, s.diagnostic());
            double target = org.firstinspires.ftc.teamcode.MecanumTeleOp.FLYWHEEL_IDLE_RPM / 60 * 28;
            for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE && v.tick >= 150)
                assertEquals(target, v.flywheelVelocity, 100.0 / 60 * 28, s.diagnostic());
            assertStopped(h);
        }
    }

    static void assertStopped(BioBuzzSimHardware h) {
        for (org.horizon36596.simloop.fakehardware.FakeMotor wheel : h.wheels) assertEquals(0, wheel.getPower(), 0.0);
        assertEquals(0, h.intake.getPower(), 0.0); assertEquals(0, h.transfer1.getPower(), 0.0);
        assertEquals(0, h.transfer2.getPower(), 0.0); assertEquals(0, h.flywheel.getPower(), 0.0);
        assertEquals(.5, h.gate.getPosition());
    }
}
