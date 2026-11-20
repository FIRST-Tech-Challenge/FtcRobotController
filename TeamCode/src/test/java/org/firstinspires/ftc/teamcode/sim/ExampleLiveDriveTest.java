package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import org.firstinspires.ftc.teamcode.simloopexample.ExampleClaw;
import org.firstinspires.ftc.teamcode.simloopexample.ExampleRobotSim;
import org.firstinspires.ftc.teamcode.simloopexample.ExampleSlide;
import org.horizon36596.simloop.desktop.KeyboardWindow;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.horizon36596.simloop.sim.live.GamepadInputGate;
import org.horizon36596.simloop.sim.live.GamepadInputSource;
import org.horizon36596.simloop.sim.live.GamepadState;
import org.horizon36596.simloop.sim.live.KeyboardGamepadSource;
import org.horizon36596.simloop.sim.live.RealTimePacer;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.List;
import java.util.Set;

/**
 * Drives the SimLoop example robot live from the laptop keyboard and logs the same RLOG a headless test
 * would, so the session can be replayed in AdvantageScope afterwards.
 *
 * <p>This test is opt-in because it opens a window and runs in real time: a normal
 * {@code ./gradlew :TeamCode:testDebugUnitTest} must stay headless and finish in seconds. Run it with
 *
 * <pre>
 * SIMLOOP_LIVE=1 ./gradlew :TeamCode:testDebugUnitTest --tests "*LiveDrive*"
 * </pre>
 *
 * <p>On Windows (cmd) use {@code set SIMLOOP_LIVE=1} first, or prefix the Gradle command with
 * {@code set SIMLOOP_LIVE=1 &&}.
 *
 * <p>Controls:
 *
 * <ul>
 *   <li>{@code W A S D} - drive forward/back and strafe left/right</li>
 *   <li>{@code I J K L} - turn and strafe with the right stick</li>
 *   <li>{@code 1} - open/close the claw; {@code 3} - raise/lower the slide</li>
 *   <li>{@code SHIFT} - half stick travel; {@code TAB} - switch gamepad 1/2; {@code ESC} - finish</li>
 * </ul>
 */
class ExampleLiveDriveTest {

    /** One sim tick, in seconds: 0.02 s is 50 Hz, the rate an FTC OpMode loop runs at. */
    private static final double DELTA_TIME_SECONDS = 0.02;

    /** Hard limit on the session, in ticks. 3000 ticks is one minute of robot time. */
    private static final int DEFAULT_MAX_TICKS = 3000;

    /**
     * The session's tick budget: {@link #DEFAULT_MAX_TICKS}, or {@code SIMLOOP_LIVE_TICKS} when set, so a
     * smoke run can finish in a second without touching the code.
     */
    private static int maxTicks() {
        String override = System.getenv("SIMLOOP_LIVE_TICKS");
        if (override == null) {
            return DEFAULT_MAX_TICKS;
        }
        try {
            int ticks = Integer.parseInt(override);
            return ticks > 0 ? ticks : DEFAULT_MAX_TICKS;
        } catch (NumberFormatException notANumber) {
            return DEFAULT_MAX_TICKS;
        }
    }

    @Test
    void drivingWithTheKeyboardMovesTheSimulatedRobot() throws IOException {
        assumeTrue("1".equals(System.getenv("SIMLOOP_LIVE")),
                "set SIMLOOP_LIVE=1 to open the interactive keyboard window");

        FakeTimer timer = new FakeTimer();
        ExampleRobotSim sim = new ExampleRobotSim();
        Path rlogPath = Paths.get("build", "sim", "example-live-drive.rlog");

        // KeyboardWindow is in the desktop jar precisely so no AWT type reaches Android test sources.
        // It throws on a headless machine (CI), which is a skip here, not a failure.
        KeyboardWindow window;
        try {
            window = KeyboardWindow.open("SimLoop live drive - ExampleRobot",
                    List.of(
                            "W A S D   drive / strafe",
                            "I J K L   turn / strafe",
                            "1   open or close claw      3   slide up or down",
                            "SHIFT   half speed     TAB   gamepad 1/2     ESC   finish"));
        } catch (IllegalStateException headless) {
            assumeTrue(false, "the live keyboard window needs a display: " + headless.getMessage());
            return;
        }

        // Button edge-detection: the mechanisms toggle on the press, not while the key is held.
        boolean[] clawOpen = {false};
        boolean[] slideUp = {false};
        boolean[] aWasDown = {false};
        boolean[] xWasDown = {false};

        int ticks;
        try {
            KeyboardGamepadSource.Keyboard keys = new KeyboardGamepadSource.Keyboard() {
                @Override
                public Set<String> keysDown() {
                    return window.keysDown();
                }

                @Override
                public boolean isOpen() {
                    return window.isOpen();
                }
            };

            GamepadInputSource source = new KeyboardGamepadSource(keys, maxTicks());
            GamepadInputGate gate = new GamepadInputGate(source);

            ticks = ScenarioRunner.runUntil("ExampleLiveDrive", ExampleRobotSim.ROBOT_CONFIG,
                    timer, DELTA_TIME_SECONDS, rlogPath, gate, new RealTimePacer(),
                    Collections.emptyList(), deltaTime -> {
                        GamepadState pad = gate.current().gamepad1;

                        // A gamepad stick is positive right / positive down. ExampleDrive wants
                        // positive forward / positive left / positive counter-clockwise, so all three
                        // are negated here.
                        double forward = -pad.axis("left_stick_y");
                        double strafe = -pad.axis("left_stick_x");
                        double turn = -pad.axis("right_stick_x");
                        sim.robot.drive.drive(forward, strafe, turn);

                        boolean aDown = pad.isDown("a");
                        if (aDown && !aWasDown[0]) {
                            clawOpen[0] = !clawOpen[0];
                            sim.robot.claw.setState(
                                    clawOpen[0] ? ExampleClaw.State.OPEN : ExampleClaw.State.CLOSED);
                        }
                        aWasDown[0] = aDown;

                        boolean xDown = pad.isDown("x");
                        if (xDown && !xWasDown[0]) {
                            slideUp[0] = !slideUp[0];
                            sim.robot.slide.setLevel(
                                    slideUp[0] ? ExampleSlide.Level.SCORE : ExampleSlide.Level.FLOOR);
                        }
                        xWasDown[0] = xDown;

                        // The once-per-loop contract every subsystem and plant expects.
                        sim.robot.periodic();
                        sim.advancePhysics(deltaTime);

                        // The same pose key the recorded example writes, so one AdvantageScope layout
                        // works for both the scripted auto and a live session.
                        sim.logPose("View/robotPose");
                        Logger.recordOutput("View/forwardCommand", forward);
                        Logger.recordOutput("View/strafeCommand", strafe);
                        Logger.recordOutput("View/turnCommand", turn);
                        Logger.recordOutput("View/slideHeightInches",
                                sim.robot.slide.getPositionInches());
                        Logger.recordOutput("View/clawCommanded",
                                sim.robot.claw.getCommandedPosition());
                    });
        } finally {
            window.close();
        }

        assertTrue(ticks > 0, "the live session should run at least one tick");
    }
}
