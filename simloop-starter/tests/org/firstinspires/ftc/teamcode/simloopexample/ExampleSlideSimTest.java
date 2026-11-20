package org.firstinspires.ftc.teamcode.simloopexample;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.loop.RunResult;
import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * The first test to read. It drives {@link ExampleSlide} - the real subsystem, not a stand-in - to a
 * scoring height and checks it gets there and stays.
 *
 * <p>Nothing here reaches into the subsystem. The assertions are read back out of the log the run wrote,
 * which is the same shape of file a real robot writes, so the same assertions would work against a run
 * recorded on the field.
 */
class ExampleSlideSimTest {

    /** Length of one tick, in seconds. 0.02 s is 50 Hz, the rate an FTC OpMode loop is written for. */
    private static final double DELTA_TIME_SECONDS = 0.02;

    /** How long the scenario runs, in ticks. 150 ticks at 0.02 s is three seconds of robot time. */
    private static final int TICKS = 150;

    /**
     * The key the slide's height is READ BACK under, which is not the key it was written under. PsiKit
     * files everything {@code Logger.recordOutput} writes beneath {@code RealOutputs/}, so a run that
     * logged {@code "Slide/heightInches"} is queried as {@code "RealOutputs/Slide/heightInches"}.
     * Getting this wrong throws rather than returning zero, on purpose.
     */
    private static final String SLIDE_HEIGHT = "RealOutputs/Slide/heightInches";

    @Test
    void theSlideReachesScoringHeightAndHoldsIt() throws IOException {
        // The clock. Everything on the sim side asks this rather than the wall, which is what makes the
        // run reproduce byte for byte.
        FakeTimer timer = new FakeTimer();

        // The robot, its fake hardware and the physics behind it.
        ExampleRobotSim sim = new ExampleRobotSim();

        // Where the log goes. build/ is gitignored, which is where a run belongs.
        Path rlogPath = Paths.get("build", "sim", "example-slide-reaches-score.rlog");

        sim.robot.slide.setLevel(ExampleSlide.Level.SCORE);

        ScenarioRunner.run("ExampleSlide", ExampleRobotSim.ROBOT_CONFIG, timer, TICKS,
                DELTA_TIME_SECONDS, rlogPath, deltaTime -> {
                    // 1. Robot code, unchanged. This is the same call an OpMode's loop() makes.
                    sim.robot.periodic();

                    // 2. The world moves, and the encoder learns what happened.
                    sim.advancePhysics(deltaTime);

                    // 3. Log what you want to assert on and what you want to look at in AdvantageScope.
                    //    Read through the subsystem, not the plant: this is the number the robot itself
                    //    believes, so an assertion on it is an assertion about robot code.
                    Logger.recordOutput("Slide/heightInches", sim.robot.slide.getPositionInches());
                    Logger.recordOutput("Slide/commandedPower", sim.robot.slide.getCommandedPower());
                });

        RunResult run = RunResult.fromRlog("ExampleSlide", rlogPath, TICKS);

        assertEquals(ExampleSlide.Level.SCORE.heightInches(), run.finalValue(SLIDE_HEIGHT),
                ExampleSlide.TOLERANCE_INCHES,
                "the slide should finish within its own tolerance of the scoring height");

        // Overshoot, not the end stop. Asserting the carriage stayed between 0 and 24 inches would pass
        // no matter what the controller did, because the plant clamps to those every tick - that is the
        // model's guarantee, not the controller's achievement. Overshoot is what this gain can actually
        // get wrong.
        assertTrue(run.maxValue(SLIDE_HEIGHT) <= ExampleSlide.Level.SCORE.heightInches() + 2.0,
                "the slide overshot scoring height by more than two inches: " + run.maxValue(SLIDE_HEIGHT));
    }
}
