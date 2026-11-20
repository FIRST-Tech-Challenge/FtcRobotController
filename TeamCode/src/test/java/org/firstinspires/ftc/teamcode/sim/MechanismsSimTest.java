package org.firstinspires.ftc.teamcode.sim;

import org.junit.jupiter.api.Test;
import org.horizon36596.simloop.opmode.OpModePhase;
import static org.junit.jupiter.api.Assertions.*;

class MechanismsSimTest {
    @Test void independentLightsUseSpecifiedPulsesAndTurnOffAtStop() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withLights();
        TeleOpScenario s = new TeleOpScenario("lights", "LightsTestOpMode", h, .02, 3, 60, (p,t,a,b) -> {
            boolean active = p == OpModePhase.ACTIVE;
            a.a = active && t < 10;
            a.dpad_right = active && ((t >= 10 && t < 20) || (t >= 30 && t < 40));
            a.b = active && t >= 10 && t < 20;
            a.x = active && t >= 30 && t < 40;
        });
        s.observer = v -> {
            if (v.phase == OpModePhase.ACTIVE && v.tick == 45) {
                assertEquals(1500, h.ledIntake.getPulseWidth(), 1);
                assertEquals(1100, h.ledShooter.getPulseWidth(), 1);
                assertEquals(1700, h.ledStatus.getPulseWidth(), 1);
            }
        };
        s.run();
        assertEquals(600, h.ledIntake.getPulseWidth(), 1);
        assertEquals(600, h.ledShooter.getPulseWidth(), 1);
        assertEquals(600, h.ledStatus.getPulseWidth(), 1);
    }
    @Test void intakePrioritiesAndStop() throws Exception {
        TeleOpScenario s = new TeleOpScenario("intake", "IntakeTestOpMode",
                new BioBuzzSimHardware().withIntake(), .02, 5, 80, (p,t,a,b) -> {
            a.right_bumper = p == OpModePhase.ACTIVE && t < 40;
            a.left_bumper = p == OpModePhase.ACTIVE && t >= 20 && t < 60;
        });
        s.run();
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 10).intakePower);
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 30).intakePower);
        assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 50).intakePower);
        assertEquals(0, s.hardware.intake.getPower());
    }

    // Stage 1 loading (like shot feeding) is open-loop full power; it never enters velocity control.
    static void assertLoading(TeleOpTrace.Sample v, String diagnostic) {
        assertFalse(v.transfer1VelocityMode, diagnostic);
        assertEquals(1, v.transfer1Power, 0, diagnostic);
    }

    @Test void transferFullDwellTimedFeedAndInhibit() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withTransfer();
        TeleOpScenario s = new TeleOpScenario("transfer", "TransferTestOpMode", h, .02, 5, 230, (p,t,a,b) -> {
            h.distance.setSimulatedInputState(p == OpModePhase.ACTIVE && t < 130);
            a.right_bumper = p == OpModePhase.ACTIVE && t >= 140;
            a.left_bumper = p == OpModePhase.ACTIVE && t >= 210;
        });
        s.run();
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 20), s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 20).transfer2Power); // Linked motor 2 loads with stage 1.
        // 0.5 s HIGH (about tick 25) confirms full; stage 1 runs on 0.5 s (to about tick 50), then stops.
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 35), s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 45), s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 55).transfer1Power);
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 120).transfer1Power);
        assertTrue(s.trace.samples.stream().anyMatch(TeleOpTrace.Sample::feedingThroughOpenGate));
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 215).transfer1Power);
        assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 215).gatePosition);
        assertEquals(0, h.transfer1.getPower()); assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }

    @Test void depollinatorEdgesHomePriorityAndPwm() throws Exception {
        TeleOpScenario s = new TeleOpScenario("depollinator", "DepollinatorTestOpMode",
                new BioBuzzSimHardware().withDepollinator(), .02, 5, 60, (p,t,a,b) -> {
            a.right_stick_button = p == OpModePhase.ACTIVE && t >= 10;
            a.left_stick_button = p == OpModePhase.ACTIVE && t >= 30;
        });
        s.run();
        assertEquals(165.0/355, s.trace.at(OpModePhase.ACTIVE, 20).depollinator, 1e-9);
        assertEquals(30.0/355, s.trace.at(OpModePhase.ACTIVE, 40).depollinator, 1e-9);
        assertEquals(Math.round(500 + 2000 * 30.0/355), s.hardware.depollinator.getPulseWidth(), 1e-6);
    }

    @Test void linearSlideTogglesAcrossAnalogWrap() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withSlide();
        TeleOpScenario s = new TeleOpScenario("linear-slide", "LinearSlideTestOpMode", h, .02, 5, 340, (p,t,a,b) -> {
            b.right_stick_button = p == OpModePhase.ACTIVE && t >= 10 && t < 13;   // extend
            b.left_stick_button = p == OpModePhase.ACTIVE && t >= 160 && t < 163;  // home
        });
        s.run();
        // INIT only tracks; no slide power before START.
        assertTrue(s.trace.samples.stream().filter(v -> v.phase == OpModePhase.INIT || v.phase == OpModePhase.INIT_LOOP).allMatch(v -> v.slidePower == 0), s.diagnostic());
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 15).slidePower > 0, s.diagnostic());
        // 45.6 deg raw down, angle falling: the 700 deg raise crosses the analog 360/0 wrap twice.
        TeleOpTrace.Sample up = s.trace.at(OpModePhase.ACTIVE, 150);
        assertEquals(700, up.slideTravelDeg, 3, s.diagnostic());
        assertEquals(0, up.slidePower, 0, s.diagnostic());
        assertTrue(up.telemetry.contains("Slide (GP2 RS/LS) : UP"), up.telemetry);
        TeleOpTrace.Sample down = s.trace.at(OpModePhase.ACTIVE, 330);
        assertEquals(0, down.slideTravelDeg, 3, s.diagnostic());
        assertEquals(0, down.slidePower, 0, s.diagnostic());
        assertTrue(down.telemetry.contains("Slide (GP2 RS/LS) : DOWN"), down.telemetry);
        assertEquals(0, h.slideServo.getPower(), 0);
    }

    @Test void jammedLinearSlideFaultsWithZeroPower() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withSlide();
        TeleOpScenario s = new TeleOpScenario("linear-slide-jam", "LinearSlideTestOpMode", h, .02, 3, 80, (p,t,a,b) -> {
            b.right_stick_button = p == OpModePhase.ACTIVE && t >= 10 && t < 13;
        });
        s.model.jamSlide = true; // explicit mechanical jam: power produces no shaft motion
        s.run();
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 15).slidePower > 0, s.diagnostic());
        TeleOpTrace.Sample faulted = s.trace.at(OpModePhase.ACTIVE, 70);
        assertEquals(0, faulted.slidePower, 0, s.diagnostic());
        assertTrue(faulted.telemetry.contains("Slide (GP2 RS/LS) : FAULT: stalled"), faulted.telemetry);
    }

    @Test void turretIndependentEncoderHoodEdgesAndFlywheel() throws Exception {
        TeleOpScenario s = new TeleOpScenario("turret", "TurretTestOpMode",
                new BioBuzzSimHardware().withTurret(), .02, 160, 160, (p,t,a,b) -> {
            a.right_stick_x = p == OpModePhase.ACTIVE && t < 60 ? .5f : 0;
            a.right_bumper = p == OpModePhase.ACTIVE && t < 110;
            a.dpad_up = p == OpModePhase.ACTIVE && t < 60;
        });
        s.run();
        assertTrue(Math.abs(s.trace.at(OpModePhase.ACTIVE, 60).yaw) > 5, s.diagnostic());
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 90).flywheelVelocity > 1000, s.diagnostic());
        // Installed home is 13 hood degrees below the original reference; held D-pad adds one 0.5-degree step.
        assertEquals(.5 + (13.0 + .5) * 2 / 355, s.trace.at(OpModePhase.ACTIVE, 40).hood, 1e-8);
        assertEquals(s.hardware.yaw1.getPosition(), s.hardware.yaw2.getPosition());
        assertEquals(0, s.hardware.flywheel.getPower());
    }

    @Test void benchActualOpModeStartsAndStopsSafely() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        double savedRpm = org.firstinspires.ftc.teamcode.IntakeShooterTestOpMode.TARGET_RPM;
        try {
            double baseRpm = Math.round(savedRpm);
            org.firstinspires.ftc.teamcode.IntakeShooterTestOpMode.TARGET_RPM = baseRpm;
            TeleOpScenario s = new TeleOpScenario("bench", "IntakeShooterTestOpMode", h, .02, 160, 100,
                    (p,t,a,b) -> {
                boolean active = p == OpModePhase.ACTIVE;
                a.right_bumper = active && t > 10 && t < 60;
                // Fine RPM: two D-pad Right presses (+10 each) and one D-pad Left press (-10).
                a.dpad_right = active && ((t >= 70 && t < 74) || (t >= 80 && t < 84));
                a.dpad_left = active && t >= 90 && t < 94;
            });
            s.run();
            assertFalse(s.result.endedBeforeStop(), s.diagnostic());
            assertTrue(s.trace.samples.stream().anyMatch(v -> v.intakePower != 0));
            assertLoading(s.trace.at(OpModePhase.ACTIVE, 30), s.diagnostic());
            assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 30).transfer2Power, s.diagnostic()); // Linked to stage 1.
            assertLoading(s.trace.at(OpModePhase.ACTIVE, 70), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 76).telemetry.contains(
                    String.format("Selected flywheel (RPM) : %.0f", baseRpm + 10)), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 86).telemetry.contains(
                    String.format("Selected flywheel (RPM) : %.0f", baseRpm + 20)), s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, 96).telemetry.contains(
                    String.format("Selected flywheel (RPM) : %.0f", baseRpm + 10)), s.diagnostic());
            assertEquals(0, h.intake.getPower()); assertEquals(0, h.flywheel.getPower());
            assertEquals(0, h.transfer1.getPower()); assertEquals(.5, h.gate.getPosition());
        } finally {
            org.firstinspires.ftc.teamcode.IntakeShooterTestOpMode.TARGET_RPM = savedRpm;
        }
    }

    @Test void benchLoadsHoldsFullFeedsAndReloadsThroughActualMotorOutputs() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        TeleOpScenario s = new TeleOpScenario("bench-transfer-cycle", "IntakeShooterTestOpMode",
                h, .02, 160, 650, (p,t,a,b) -> {
            boolean active = p == OpModePhase.ACTIVE;
            a.right_bumper = active && ((t >= 10 && t < 70) || (t >= 110 && t < 130) || (t >= 210 && t < 230) || t >= 600);
            a.left_bumper = active && t >= 80 && t < 130;
            h.distance.setSimulatedInputState(active && t >= 140 && t < 270);
            a.right_trigger = active && t >= 340 && t < 570 ? 1 : 0;
            a.left_trigger = active && t >= 420 && t < 570 ? 1 : 0;
        });
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 30).intakePower, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 30), s.diagnostic());
        assertEquals(-1, s.trace.at(OpModePhase.ACTIVE, 90).intakePower, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 90).transfer1Power, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 120).intakePower, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 120).transfer1Power, s.diagnostic());
        // Confirmed full (HIGH since tick 140, confirmed about tick 165) stops the intake instead of
        // reversing it, while stage 1 runs on until about tick 190 to seat the fourth ball.
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 175).intakePower, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 175), s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 175).transfer2Power, s.diagnostic()); // Run-on on both motors.
        // RB while HIGH/full forces the intake and (competition override) stage 1 forward.
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 220).intakePower, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 220), s.diagnostic());
        for (int tick : new int[]{200, 250}) {
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).transfer1Power, s.diagnostic());
        }
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 275).intakePower, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 275), s.diagnostic()); // LOW reloads without feed.
        // RT alone spins the flywheel without pausing collection.
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 400).intakePower, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 400), s.diagnostic());
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE
                && v.tick >= 420 && v.tick < 570 && v.feedingThroughOpenGate() && v.transfer2Power == 1), s.diagnostic());
        // Bench-only gate delay: each time the gate opens, both linked transfer motors stay off for the
        // 0.10 s gate-open-to-transfer delay (5 ticks), then feed at full power. The tick that lands exactly on
        // the delay boundary may go either way.
        double delay = org.firstinspires.ftc.teamcode.IntakeShooterTestController.GATE_OPEN_TO_TRANSFER_SECONDS;
        double gateOpenedAt = Double.NaN;
        int delayedTicks = 0, feedTicks = 0;
        for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
            if (!v.gateOpen()) gateOpenedAt = Double.NaN;
            else {
                if (Double.isNaN(gateOpenedAt)) gateOpenedAt = v.seconds;
                double sinceOpen = v.seconds - gateOpenedAt;
                assertEquals(0, v.intakePower, s.diagnostic()); // Automatic collection pauses while shooting.
                if (sinceOpen < delay - 1e-6) {
                    assertEquals(0, v.transfer1Power, s.diagnostic());
                    assertEquals(0, v.transfer2Power, s.diagnostic());
                    delayedTicks++;
                } else if (sinceOpen > delay + .02 - 1e-6) {
                    assertEquals(1, v.transfer1Power, s.diagnostic());
                    assertEquals(1, v.transfer2Power, s.diagnostic());
                    feedTicks++;
                }
            }
            if (v.tick < 420 || v.tick >= 570) assertFalse(v.feedingThroughOpenGate(), s.diagnostic());
        }
        assertTrue(delayedTicks >= 5, s.diagnostic());
        assertTrue(feedTicks > 0, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 580), s.diagnostic());
        assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 580).gatePosition, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 620).intakePower, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 620), s.diagnostic());
        assertEquals(0, h.intake.getPower()); assertEquals(0, h.flywheel.getPower());
        assertEquals(0, h.transfer1.getPower()); assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }

    @Test void benchManualFeedAndInhibitFollowCompetitionThroughActualOpMode() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        TeleOpScenario s = new TeleOpScenario("bench-manual-feed", "IntakeShooterTestOpMode",
                h, .02, 160, 180, (p,t,a,b) -> {
            boolean active = p == OpModePhase.ACTIVE;
            b.right_trigger = active && t >= 20 && t < 100 ? 1 : 0; // GP2 manual feed.
            a.b = active && t >= 110 && t < 150; // Bench-only GP1 B manual feed.
            b.left_trigger = active && t >= 60 && t < 70 ? 1 : 0; // GP2 inhibit.
            a.back = active && t >= 140 && t < 145; // Bench stop.
        });
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
        // RT is never held, so the bench flywheel stays off: manual feed bypasses readiness. The gate opens at
        // ticks 20 (GP2 RT), 70 (inhibit released) and 110 (GP1 B); the transfer waits the bench-only 0.10 s.
        for (int tick : new int[]{20, 21, 24, 70, 74, 110, 114}) {
            TeleOpTrace.Sample v = s.trace.at(OpModePhase.ACTIVE, tick);
            assertEquals(.2, v.gatePosition, 1e-9, s.diagnostic());
            assertEquals(0, v.transfer1Power, s.diagnostic());
            assertEquals(0, v.transfer2Power, s.diagnostic());
            assertEquals(0, v.intakePower, s.diagnostic());
        }
        for (int tick : new int[]{26, 59, 76, 99, 116, 139}) {
            TeleOpTrace.Sample v = s.trace.at(OpModePhase.ACTIVE, tick);
            assertEquals(.2, v.gatePosition, 1e-9, s.diagnostic());
            assertEquals(1, v.transfer1Power, s.diagnostic());
            assertFalse(v.transfer1VelocityMode, s.diagnostic()); // Shots feed at full open-loop power.
            assertEquals(1, v.transfer2Power, s.diagnostic());
            assertEquals(0, v.intakePower, s.diagnostic());
        }
        for (int tick : new int[]{62, 68, 100, 105, 140, 145, 170}) {
            TeleOpTrace.Sample v = s.trace.at(OpModePhase.ACTIVE, tick);
            assertEquals(.5, v.gatePosition, 1e-9, s.diagnostic());
            // Released manual feed and a re-armed bench resume loading; inhibit, Back and unarmed pause it.
            if (tick == 100 || tick == 105 || tick == 170) assertLoading(v, s.diagnostic());
            else assertEquals(0, v.transfer1Power, s.diagnostic());
            assertEquals(v.transfer1Power, v.transfer2Power, s.diagnostic());
        }
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 64).telemetry.contains("BLOCKED (GP2 LT)"), s.diagnostic());
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 120).telemetry.contains("MANUAL FEED (GP1 B / GP2 RT)"), s.diagnostic());
        // No bench idle: armed with RT released the flywheel stays off and never leaves rest.
        for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
            assertEquals(0, v.flywheelVelocity, 1e-6, s.diagnostic());
            assertTrue(v.telemetry.contains("Fusion zone / power : OFF"), s.diagnostic());
        }
        assertEquals(0, h.transfer1.getPower());
        assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }

    /** Wiring and interlocks only: the flywheel fixture has synthetic response and no friction or ball load. */
    @Test void benchFusionControllerSpinsUpHoldsAndFeedsThroughActualOpMode() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        TeleOpScenario s = new TeleOpScenario("bench-fusion", "IntakeShooterTestOpMode",
                h, .02, 160, 300, (p,t,a,b) -> {
            boolean active = p == OpModePhase.ACTIVE;
            a.right_trigger = active && t >= 40 && t < 260 ? 1 : 0;
            a.left_trigger = active && t >= 120 && t < 260 ? 1 : 0;
            b.b = active && t >= 270 && t < 276; // One held press: fusion -> PIDF exactly once.
        });
        s.observer = v -> {
            if (v.phase == OpModePhase.ACTIVE && v.tick == 200)
                assertEquals(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER, h.flywheel.getMode());
        };
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
        // Competition default regulation from START. No bench idle: the wheel stays off until RT (tick 40),
        // then spins up from rest at full power.
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 5).telemetry.contains("Flywheel regulation : BANG_BANG_FUSION"), s.diagnostic());
        for (int tick : new int[]{1, 20, 39}) {
            assertTrue(s.trace.at(OpModePhase.ACTIVE, tick).telemetry.contains("Fusion zone / power : OFF"), s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).flywheelVelocity, 1e-6, s.diagnostic());
        }
        // The synthetic wheel reaches the band within a few ticks, so check the first two RT ticks.
        for (int tick : new int[]{40, 41})
            assertTrue(s.trace.at(OpModePhase.ACTIVE, tick).telemetry.contains("Fusion zone / power : BANG-BANG | +100%"),
                    s.diagnostic());
        double targetTicksPerSecond = org.firstinspires.ftc.teamcode.IntakeShooterTestOpMode.TARGET_RPM / 60 * 28;
        double readyToleranceTicksPerSecond = 100.0 / 60 * 28;
        // RT released: the target drops to zero (coast), not to an idle speed.
        TeleOpTrace.Sample released = s.trace.at(OpModePhase.ACTIVE, 265);
        assertTrue(released.telemetry.contains("Fusion zone / power : OFF"), s.diagnostic());
        assertTrue(released.flywheelVelocity < s.trace.at(OpModePhase.ACTIVE, 259).flywheelVelocity, s.diagnostic());
        for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
            if (v.tick >= 100 && v.tick < 260) {
                // Held by feedforward inside the unchanged +/-100 RPM readiness tolerance, not by cycling.
                assertEquals(targetTicksPerSecond, v.flywheelVelocity, readyToleranceTicksPerSecond, s.diagnostic());
                assertTrue(v.telemetry.contains("Fusion zone / power : FEEDFORWARD"), s.diagnostic());
            }
            if (v.tick < 120 || v.tick >= 260) assertFalse(v.feedingThroughOpenGate(), s.diagnostic());
            // Overspeed coasts: the commanded power is never negative in either zone.
            assertFalse(v.telemetry.contains("Fusion zone / power : BANG-BANG | -"), s.diagnostic());
            assertFalse(v.telemetry.contains("Fusion zone / power : FEEDFORWARD | -"), s.diagnostic());
        }
        // The competition readiness interlock permits continuous feeding under fusion regulation.
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE
                && v.tick >= 120 && v.tick < 260 && v.feedingThroughOpenGate()), s.diagnostic());
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 290).telemetry.contains("Flywheel regulation : PIDF"), s.diagnostic());
        assertEquals(0, h.flywheel.getPower());
        assertEquals(0, h.transfer1.getPower());
        assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }
}
