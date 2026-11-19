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
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 20).transfer2Power);
        // 0.5 s HIGH (about tick 25) confirms full; stage 1 runs on 0.5 s (to about tick 50), then stops.
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 35), s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 45), s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 55).transfer1Power);
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 120).transfer1Power);
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.transfer2Power > 0 && Math.abs(v.gatePosition - .2) < 1e-9));
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

    @Test void linearSlideTogglesAcrossAnalogWrapAndClawToggles() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withSlide();
        TeleOpScenario s = new TeleOpScenario("linear-slide", "LinearSlideTestOpMode", h, .02, 5, 180, (p,t,a,b) -> {
            boolean press = p == OpModePhase.ACTIVE && ((t >= 10 && t < 13) || (t >= 90 && t < 93));
            b.left_stick_button = press;
            a.b = press;
        });
        s.run();
        // INIT only tracks; no slide power before START.
        assertTrue(s.trace.samples.stream().filter(v -> v.phase == OpModePhase.INIT || v.phase == OpModePhase.INIT_LOOP).allMatch(v -> v.slidePower == 0), s.diagnostic());
        assertEquals(.60, s.trace.at(OpModePhase.ACTIVE, 5).claw, 1e-9);
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 15).slidePower > 0, s.diagnostic());
        assertEquals(.40, s.trace.at(OpModePhase.ACTIVE, 15).claw, 1e-9);
        // Start shaft 300 deg + 180 crosses the analog 360/0 wrap.
        TeleOpTrace.Sample up = s.trace.at(OpModePhase.ACTIVE, 80);
        assertEquals(180, up.slideTravelDeg, 5, s.diagnostic());
        assertEquals(0, up.slidePower, 0, s.diagnostic());
        assertTrue(up.telemetry.contains("Slide (GP2 LS) : UP"), up.telemetry);
        TeleOpTrace.Sample down = s.trace.at(OpModePhase.ACTIVE, 170);
        assertEquals(0, down.slideTravelDeg, 5, s.diagnostic());
        assertEquals(0, down.slidePower, 0, s.diagnostic());
        assertEquals(.60, down.claw, 1e-9);
        assertTrue(down.telemetry.contains("Slide (GP2 LS) : DOWN"), down.telemetry);
        assertEquals(0, h.slideServo.getPower(), 0);
    }

    @Test void jammedLinearSlideFaultsWithZeroPower() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withSlide();
        TeleOpScenario s = new TeleOpScenario("linear-slide-jam", "LinearSlideTestOpMode", h, .02, 3, 80, (p,t,a,b) -> {
            b.left_stick_button = p == OpModePhase.ACTIVE && t >= 10 && t < 13;
        });
        s.model.jamSlide = true; // explicit mechanical jam: power produces no shaft motion
        s.run();
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 15).slidePower > 0, s.diagnostic());
        TeleOpTrace.Sample faulted = s.trace.at(OpModePhase.ACTIVE, 70);
        assertEquals(0, faulted.slidePower, 0, s.diagnostic());
        assertTrue(faulted.telemetry.contains("Slide (GP2 LS) : FAULT: stalled"), faulted.telemetry);
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
        // Installed home is 10.5 hood degrees below the original reference; held D-pad adds one 0.5-degree step.
        assertEquals(.5 + (10.5 + .5) * 2 / 355, s.trace.at(OpModePhase.ACTIVE, 40).hood, 1e-8);
        assertEquals(s.hardware.yaw1.getPosition(), s.hardware.yaw2.getPosition());
        assertEquals(0, s.hardware.flywheel.getPower());
    }

    @Test void benchActualOpModeStartsAndStopsSafely() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        TeleOpScenario s = new TeleOpScenario("bench", "IntakeShooterTestOpMode", h, .02, 160, 100,
                (p,t,a,b) -> a.right_bumper = p == OpModePhase.ACTIVE && t > 10 && t < 60);
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.intakePower != 0));
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 30), s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 30).transfer2Power, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 70), s.diagnostic());
        assertEquals(0, h.intake.getPower()); assertEquals(0, h.flywheel.getPower());
        assertEquals(0, h.transfer1.getPower()); assertEquals(.5, h.gate.getPosition());
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
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 175).transfer2Power, s.diagnostic());
        assertEquals(0, s.trace.at(OpModePhase.ACTIVE, 200).transfer1Power, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 220).intakePower, s.diagnostic()); // RB while HIGH/full.
        for (int tick : new int[]{200, 250}) {
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).intakePower, s.diagnostic());
            assertEquals(0, s.trace.at(OpModePhase.ACTIVE, tick).transfer1Power, s.diagnostic());
        }
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 275).intakePower, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 275), s.diagnostic()); // LOW reloads without feed.
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE
                && v.tick >= 420 && v.tick < 570 && v.transfer1Power == 1 && v.transfer2Power == 1), s.diagnostic());
        double openedAt = Double.NaN;
        for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
            if (Math.abs(v.gatePosition - .2) < 1e-9) {
                if (Double.isNaN(openedAt)) openedAt = v.seconds;
            } else openedAt = Double.NaN;
            if (v.transfer2Power > 0) {
                assertFalse(Double.isNaN(openedAt), s.diagnostic());
                double elapsed = v.seconds - openedAt;
                assertTrue(elapsed >= .10 - 1e-9, s.diagnostic());
                // Stage 1 feeds together with stage 2 for the whole gate-open feed.
                assertEquals(1, v.transfer1Power, s.diagnostic());
            }
            if (v.tick < 420 || v.tick >= 570) assertEquals(0, v.transfer2Power, s.diagnostic());
        }
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 580), s.diagnostic());
        assertEquals(.5, s.trace.at(OpModePhase.ACTIVE, 580).gatePosition, s.diagnostic());
        assertEquals(1, s.trace.at(OpModePhase.ACTIVE, 620).intakePower, s.diagnostic());
        assertLoading(s.trace.at(OpModePhase.ACTIVE, 620), s.diagnostic());
        assertEquals(0, h.intake.getPower()); assertEquals(0, h.flywheel.getPower());
        assertEquals(0, h.transfer1.getPower()); assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }

    @Test void benchContinuousFeedsImmediatelyUntilTriggerReleaseOrStop() throws Exception {
        BioBuzzSimHardware h = new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
        TeleOpScenario s = new TeleOpScenario("bench-continuous", "IntakeShooterTestOpMode",
                h, .02, 160, 180, (p,t,a,b) -> {
            boolean active = p == OpModePhase.ACTIVE;
            b.right_bumper = active && t >= 10 && t < 16;
            a.right_trigger = active && t >= 20 && (t < 120 || t >= 130) ? 1 : 0;
            a.left_trigger = active && t >= 20 && (t < 100 || t >= 110) ? 1 : 0;
            a.b = active && t >= 140 && t < 145;
        });
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
        // At tick 20 the flywheel has only just been commanded: feeding must not wait for speed or gate delay.
        for (int tick : new int[]{20, 21, 70, 99, 110, 119, 130, 139}) {
            TeleOpTrace.Sample v = s.trace.at(OpModePhase.ACTIVE, tick);
            assertEquals(.2, v.gatePosition, 1e-9, s.diagnostic());
            assertEquals(1, v.transfer1Power, s.diagnostic());
            assertFalse(v.transfer1VelocityMode, s.diagnostic()); // Shots feed at full open-loop power.
            assertEquals(1, v.transfer2Power, s.diagnostic());
        }
        for (int tick : new int[]{100, 105, 120, 125, 140, 145, 170}) {
            TeleOpTrace.Sample v = s.trace.at(OpModePhase.ACTIVE, tick);
            assertEquals(.5, v.gatePosition, 1e-9, s.diagnostic());
            // Releasing LT resumes automatic loading; B/unarmed or held LT still pauses it.
            if (tick == 100 || tick == 105) assertLoading(v, s.diagnostic());
            else assertEquals(0, v.transfer1Power, s.diagnostic());
            assertEquals(0, v.transfer2Power, s.diagnostic());
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
            b.b = active && t >= 10 && t < 16; // One held press: PIDF -> fusion exactly once.
            a.right_trigger = active && t >= 40 && t < 260 ? 1 : 0;
            a.left_trigger = active && t >= 120 && t < 260 ? 1 : 0;
        });
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 5).telemetry.contains("Flywheel regulation : PIDF"), s.diagnostic());
        TeleOpTrace.Sample switched = s.trace.at(OpModePhase.ACTIVE, 30);
        assertTrue(switched.telemetry.contains("Flywheel regulation : BANG_BANG_FUSION"), s.diagnostic());
        assertTrue(switched.telemetry.contains("Fusion zone / power : OFF"), s.diagnostic());
        assertEquals(0, switched.flywheelVelocity, s.diagnostic()); // Switching alone never spins.
        // From rest the error is far above 80 ticks/s: full forward power.
        assertTrue(s.trace.at(OpModePhase.ACTIVE, 41).telemetry.contains("Fusion zone / power : BANG-BANG | +100%"),
                s.diagnostic());
        double targetTicksPerSecond = org.firstinspires.ftc.teamcode.IntakeShooterTestOpMode.TARGET_RPM / 60 * 28;
        double readyToleranceTicksPerSecond = 100.0 / 60 * 28;
        for (TeleOpTrace.Sample v : s.trace.samples) if (v.phase == OpModePhase.ACTIVE) {
            if (v.tick >= 100 && v.tick < 260) {
                // Held by feedforward inside the unchanged +/-100 RPM readiness tolerance, not by cycling.
                assertEquals(targetTicksPerSecond, v.flywheelVelocity, readyToleranceTicksPerSecond, s.diagnostic());
                assertTrue(v.telemetry.contains("Fusion zone / power : FEEDFORWARD"), s.diagnostic());
            }
            if (v.tick < 120 || v.tick >= 260) assertEquals(0, v.transfer2Power, s.diagnostic());
            // Overspeed coasts: the commanded power is never negative in either zone.
            assertFalse(v.telemetry.contains("Fusion zone / power : BANG-BANG | -"), s.diagnostic());
            assertFalse(v.telemetry.contains("Fusion zone / power : FEEDFORWARD | -"), s.diagnostic());
        }
        // The unchanged staggered readiness interlock permits feeding under fusion regulation.
        assertTrue(s.trace.samples.stream().anyMatch(v -> v.phase == OpModePhase.ACTIVE
                && v.tick >= 120 && v.tick < 260 && v.transfer2Power == 1), s.diagnostic());
        assertEquals(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER, h.flywheel.getMode());
        assertEquals(0, h.flywheel.getPower());
        assertEquals(0, h.transfer1.getPower());
        assertEquals(0, h.transfer2.getPower());
        assertEquals(.5, h.gate.getPosition());
    }
}
