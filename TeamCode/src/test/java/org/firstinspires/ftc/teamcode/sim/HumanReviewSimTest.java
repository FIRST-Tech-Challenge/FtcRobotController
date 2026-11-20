package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Paths;
import java.nio.file.Files;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.horizon36596.simloop.desktop.KeyboardWindow;
import org.horizon36596.simloop.sim.live.*;
import org.horizon36596.simloop.opmode.OpModePhase;
import static org.junit.jupiter.api.Assertions.*;

/** Explicitly excluded from automated suites. Uses the same actual-OpMode harness and output format. */
class HumanReviewSimTest {
    static final double DT = .02;
    static final int INIT_TICKS = 160;

    private static String environmentKeys(java.util.Set<String> keys) {
        StringBuilder state = new StringBuilder();
        for (String key : new String[]{"U", "O", "P", "V"}) if (keys.contains(key)) state.append(key);
        return state.toString();
    }

    @Test void review() throws Exception {
        String replay = System.getProperty("biobuzz.sim.replay", "");
        boolean smoke = Boolean.getBoolean("biobuzz.sim.smoke");
        GamepadInputTrack recorded = replay.isEmpty() ? null : GamepadInputTrack.read(Paths.get(replay));
        String opMode = recorded == null ? System.getProperty("biobuzz.sim.opMode", "RedMecanumTeleOp")
                : recorded.metadata().get("opMode");
        if (recorded != null) assertEquals(DT, Double.parseDouble(recorded.metadata().get("tickSeconds")),
                "This reviewer requires a 20 ms recorded tick");
        int ticks = recorded == null ? (int)(Double.parseDouble(System.getProperty("biobuzz.sim.seconds", "120"))/DT)
                : recorded.frames().size();
        if (smoke) ticks = 120;
        assertTrue(ticks > 0 && ticks <= 15000, "Review duration must be >0 and <=300 seconds");
        int replayStopTick = -1;
        List<String> replayEnvironment = null;
        String profile = System.getProperty("biobuzz.sim.profile", "no-tags");
        // Sidecar is required: gamepad tracks alone cannot describe the environment or INIT.
        if (recorded != null) {
            List<String> setup = Files.readAllLines(Paths.get(replay).resolveSibling("review-setup.txt"), StandardCharsets.UTF_8);
            assertEquals("initTicks=160", setup.get(0));
            profile = setup.get(1);
            replayStopTick = Integer.parseInt(setup.get(2));
            replayEnvironment = Files.readAllLines(Paths.get(replay).resolveSibling("environment.tsv"), StandardCharsets.UTF_8);
            assertEquals(ticks, replayEnvironment.size(), "Missing environment frames");
        }
        BioBuzzSimHardware h = BioBuzzSimHardware.forOpMode(opMode);
        if (profile.startsWith("raised-audience") && h.camera != null) h.withCamera(BioBuzzSimHardware.raisedAudienceTags(), !profile.equals("raised-audience-inverted"));
        else assertEquals("no-tags", profile, "Raised-audience needs a camera-equipped OpMode");
        final KeyboardWindow window = recorded == null && !smoke ? KeyboardWindow.open("BioBuzz: " + opMode,
                Arrays.asList("WASD drive | JL turn | Z/C triggers | Q/E bumpers | 1/2/3/4 A/B/X/Y",
                        "Arrows: D-pad | F/H: stick clicks | TAB: gamepad 1/2 | SHIFT: half travel",
                        "U: hold queue sensor HIGH | O: camera outage | P: Pinpoint fault | V: turret jam",
                        "ESC: stop | Live values below; full telemetry in trace.tsv after STOP",
                        "INIT runs 3.2 simulated seconds, then START. Synthetic physics; no ball scoring.")) : null;
        try {
            GamepadInputSource source = recorded != null ? recorded.replay() : smoke
                    ? tick -> tick < 80 ? new GamepadInputTrack.Frame(tick,
                            GamepadState.of(java.util.Collections.singletonMap("left_stick_y", -.5f),
                                    java.util.Collections.emptySet()), GamepadState.AT_REST) : null
                    : new KeyboardGamepadSource(new KeyboardGamepadSource.Keyboard() {
                        public java.util.Set<String> keysDown() { return window.keysDown(); }
                        public boolean isOpen() { return window.isOpen(); }
                    }, ticks);
            final TeleOpScenario[] holder = new TeleOpScenario[1];
            final int stopOnReplay = replayStopTick;
            final int[] stoppedAt = {-1};
            final List<String> environmentInput = replayEnvironment;
            final List<String> environmentOutput = new java.util.ArrayList<>();
            TeleOpScenario s = new TeleOpScenario("review-" + opMode, opMode, h, DT, INIT_TICKS, ticks,
                    (phase,t,a,b) -> {
                        if (phase != OpModePhase.ACTIVE) return;
                        String environment = environmentInput != null ? environmentInput.get(t) : window != null
                                ? environmentKeys(window.keysDown()) : smoke && t < 60 ? "[U]" : "[]";
                        environmentOutput.add(environment);
                        if (h.distance != null) h.distance.setSimulatedInputState(environment.contains("U"));
                        if (h.camera != null) { if (environment.contains("O")) h.camera.stop(); else if (!h.camera.isRunning()) h.camera.start(); }
                        if (h.pinpoint != null) { if (environment.contains("P")) h.pinpoint.forceFault(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.NOT_READY); else h.pinpoint.clearFault(); }
                        holder[0].model.jamTurret = environment.contains("V");
                        if (stoppedAt[0] >= 0) { a.reset(); b.reset(); return; }
                        GamepadInputTrack.Frame frame = t == stopOnReplay ? null : source.next(t);
                        if (frame == null) {
                            a.reset(); b.reset(); stoppedAt[0] = t; holder[0].runner.opMode().requestOpModeStop();
                        } else { frame.gamepad1.applyTo(a); frame.gamepad2.applyTo(b); }
                    });
            holder[0] = s;
            if (profile.startsWith("raised-audience")) {
                s.model.drive.setPose(opMode.startsWith("Blue") ? 12.75 : -12.75, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES, Math.PI);
                h.pinpoint.attachChassisAt(s.model.drive.getX()*25.4, BioBuzzSimHardware.RAISED_TAG_VIEW_Y_INCHES*25.4, Math.PI);
            }
            RealTimePacer pacer = new RealTimePacer();
            s.observer = v -> {
                if (window != null && stoppedAt[0] < 0) {
                    pacer.afterTick(s.trace.samples.size(), s.trace.samples.size() * DT);
                    if (v.tick % 5 == 0) window.showStatus(String.format(java.util.Locale.ROOT,
                            "%s %d | XY %.1f,%.1f in | yaw %.1f deg | flywheel %.0f ticks/s | gate %.2f | intake %.1f | transfer %.1f",
                            v.phase, v.tick, v.xIn, v.yIn, v.yaw, v.flywheelVelocity, v.gatePosition, v.intakePower, v.transfer1Power));
                }
            };
            try { s.run(); } finally {
                Files.write(s.trace.directory.resolve("review-setup.txt"), Arrays.asList("initTicks=160", profile, Integer.toString(stoppedAt[0])), StandardCharsets.UTF_8);
                Files.write(s.trace.directory.resolve("environment.tsv"), environmentOutput, StandardCharsets.UTF_8);
            }
            if (recorded != null) {
                List<String> expected = Files.readAllLines(Paths.get(replay).resolveSibling("trace.tsv"), StandardCharsets.UTF_8);
                List<String> actual = Files.readAllLines(s.trace.directory.resolve("trace.tsv"), StandardCharsets.UTF_8);
                assertEquals(expected, actual, "Replay outputs changed; inspect traces (expected when robot code changes)");
            }
            // A reviewer stopping early is expected, not an automatic behavior verdict.
            assertTrue(s.result.warnings().isEmpty(), s.diagnostic());
            System.out.println("Review recorded. Runner completion is NOT a behavior approval. " + s.diagnostic());
        } finally { if (window != null) window.close(); }
    }
}
