package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Proxy;
import org.horizon36596.simloop.opmode.GamepadScript;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class GateSimTest {
    private TeleOpScenario gate(String name, double dt, int ticks, GamepadScript script) throws Exception {
        TeleOpScenario s = new TeleOpScenario(name, "GateTestOpMode", BioBuzzSimHardware.gate(), dt, 2, ticks, script);
        s.run();
        assertFalse(s.result.endedBeforeStop(), s.diagnostic());
        assertTrue(s.result.warnings().isEmpty(), s.result.warnings() + "; " + s.diagnostic());
        assertEquals(1 + ticks, s.result.telemetry().transmissions().size(), s.diagnostic());
        assertEquals(1500, s.trace.at(OpModePhase.INIT, 0).gatePulseUs, 0, s.diagnostic());
        assertEquals(1500, s.trace.at(OpModePhase.STOP, 0).gatePulseUs, 0, s.diagnostic());
        assertEquals(Servo.Direction.REVERSE, s.hardware.gate.getDirection());
        PwmControl.PwmRange range = s.hardware.gate.getPwmRange();
        assertEquals(500, range.usPulseLower, 0);
        assertEquals(2500, range.usPulseUpper, 0);
        assertTrue(s.hardware.gate.isPwmEnabled());
        return s;
    }

    @Test void physicalPwmAndStrictTriggerThreshold() throws Exception {
        TeleOpScenario s = gate("gate-pwm", .02, 3, (p, t, g, g2) ->
                g.left_trigger = t == 0 ? .099f : t == 1 ? .101f : 0);
        assertEquals(1500, s.trace.at(OpModePhase.ACTIVE, 0).gatePulseUs, 0);
        TeleOpTrace.Sample opened = s.trace.at(OpModePhase.ACTIVE, 1);
        assertEquals(.2, opened.gatePosition, 1e-9, s.diagnostic());
        assertEquals(.8, opened.gatePhysical, 1e-9, s.diagnostic());
        assertEquals(2100, opened.gatePulseUs, 0, s.diagnostic());
        assertFalse(opened.feedAllowed);
        assertEquals(1500, s.trace.at(OpModePhase.ACTIVE, 2).gatePulseUs, 0);
    }

    @Test void repeatedOpenDoesNotRestartDelayButCloseReopenDoes() throws Exception {
        for (double dt : new double[] {.02, .01}) {
            int delayTicks = (int) Math.round(
                    org.firstinspires.ftc.teamcode.GateSubsystem.FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS / dt);
            int closeTick = delayTicks + 3, reopenTick = closeTick + 1;
            TeleOpScenario s = gate("gate-delay", dt, reopenTick + delayTicks + 2, (p, t, g, g2) ->
                    g.left_trigger = t == closeTick ? 0 : 1);
            assertFalse(s.trace.at(OpModePhase.ACTIVE, delayTicks - 1).feedAllowed, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, delayTicks).feedAllowed, s.diagnostic());
            assertFalse(s.trace.at(OpModePhase.ACTIVE, closeTick).feedAllowed, s.diagnostic());
            assertFalse(s.trace.at(OpModePhase.ACTIVE, reopenTick).feedAllowed, s.diagnostic());
            assertFalse(s.trace.at(OpModePhase.ACTIVE, reopenTick + delayTicks - 1).feedAllowed, s.diagnostic());
            assertTrue(s.trace.at(OpModePhase.ACTIVE, reopenTick + delayTicks).feedAllowed, s.diagnostic());
        }
    }

    @Test void stopWhileOpenAndBeforeStartClosesGate() throws Exception {
        TeleOpScenario open = gate("gate-stop-open", .02, 4, (p, t, g, g2) -> g.left_trigger = 1);
        assertEquals(2100, open.trace.at(OpModePhase.ACTIVE, 3).gatePulseUs, 0);
        gate("gate-stop-before-start", .02, 0, (p, t, g, g2) -> g.left_trigger = 1);
    }

    @Test void missingGateAndServoWithoutPwmCapabilityHaveClearInitErrors() throws Exception {
        TeleOpScenario missing = new TeleOpScenario("missing-gate", "GateTestOpMode",
                new BioBuzzSimHardware(), .02, 1, 1, GamepadScript.NONE);
        RuntimeException absent = assertThrows(RuntimeException.class, missing::run);
        assertTrue(DrivetrainSimTest.causes(absent).contains("gate_servo"));

        BioBuzzSimHardware unsupported = new BioBuzzSimHardware();
        Servo servoOnly = (Servo) Proxy.newProxyInstance(getClass().getClassLoader(), new Class<?>[] {Servo.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("hashCode")) return System.identityHashCode(proxy);
                    if (method.getName().equals("equals")) return proxy == args[0];
                    if (method.getName().equals("getDeviceName")) return "Non-PWM servo fixture";
                    return null;
                });
        unsupported.map.servo.put("gate_servo", servoOnly);
        TeleOpScenario noPwm = new TeleOpScenario("gate-no-pwm", "GateTestOpMode", unsupported,
                .02, 1, 1, GamepadScript.NONE);
        RuntimeException failure = assertThrows(RuntimeException.class, noPwm::run);
        assertTrue(DrivetrainSimTest.causes(failure).contains("gate_servo must support PwmControl"),
                DrivetrainSimTest.causes(failure));
    }
}
