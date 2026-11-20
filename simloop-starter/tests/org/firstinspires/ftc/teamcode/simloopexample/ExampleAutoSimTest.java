package org.firstinspires.ftc.teamcode.simloopexample;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.horizon36596.simloop.sim.FakeTimer;
import org.horizon36596.simloop.sim.ScenarioRunner;
import org.junit.jupiter.api.Test;
import org.psilynx.psikit.core.Logger;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * A short autonomous routine, and the two things about it that a simulation can catch and a bench test
 * cannot.
 *
 * <p>The routine drives forward, raises the slide to scoring height, waits, and opens the claw. Written
 * as a tiny state machine, because that is how an autonomous is written and because a routine written
 * as a straight line of blocking sleeps cannot be stepped one tick at a time.
 *
 * <p>The assertions at the bottom are the point:
 *
 * <ul>
 *   <li><b>The chassis ends up somewhere sensible.</b> Loose bounds on purpose - the useful failure here
 *       is a drivetrain that strafes when told to go straight or turns when told to go nowhere, and a
 *       tight bound on distance would only make the test fail whenever anyone touched a gear ratio.</li>
 *   <li><b>The claw's wait is long enough.</b> {@link ExampleClaw#TRAVEL_TIME_SECONDS} is a guess a team
 *       makes once. This test reads that constant rather than repeating it, so shortening it there fails
 *       here - which on the field would have been a sample released before the jaws were open.</li>
 * </ul>
 */
class ExampleAutoSimTest {

    /** Length of one tick, in seconds. */
    private static final double DELTA_TIME_SECONDS = 0.02;

    /**
     * How long the scenario runs, in ticks. 250 ticks at 0.02 s is five seconds of robot time.
     *
     * <p>Sized with room to spare, and measured rather than guessed: the routine finishes on tick 112,
     * which is 2.24 s - one second driving, then the slide to 18 inches, then the claw's 0.45 s. The
     * assertion at the bottom holds that number to three quarters of the budget, so this paragraph
     * cannot quietly stop being true.
     * The slack is there because a scenario that runs out of ticks fails with "the routine never
     * finished", which says nothing about what was actually slow. If a change to a gain or a plant makes
     * the routine genuinely slower, the right fix is to find out why, not to raise this number.
     */
    private static final int TICKS = 250;

    /** How long the routine drives forward for, in seconds. */
    private static final double DRIVE_SECONDS = 1.0;

    /** Fraction of full effort the routine drives forward at, unitless. */
    private static final double DRIVE_POWER = 0.6;

    /** The steps of the routine, in the order they run. */
    private enum Step {
        /** Driving forward, away from the wall. */
        DRIVING,
        /** Stopped, raising the slide to scoring height. */
        LIFTING,
        /** Slide up, claw commanded open, waiting for the jaws to travel. */
        RELEASING,
        /** Done. */
        FINISHED
    }

    @Test
    void theAutonomousRoutineScoresWithoutDroppingTheSample() throws IOException {
        FakeTimer timer = new FakeTimer();
        ExampleRobotSim sim = new ExampleRobotSim();
        Path rlogPath = Paths.get("build", "sim", "example-auto.rlog");

        // The routine's own state, held in one-element arrays because a lambda may only read the
        // variables around it, not reassign them. A real OpMode holds these as fields.
        Step[] step = {Step.DRIVING};
        double[] elapsedInStep = {0.0};

        // What the jaws were really doing when the routine finished, recorded so the assertion below can
        // look at it. The robot itself has no way to know this - a bare servo has no sensor - which is
        // exactly why it takes a simulation to check the wait.
        double[] jawsAtFinish = {Double.NaN};

        // Which tick the routine finished on. Asserted at the bottom rather than only described in a
        // comment, so the tick budget above is a checked number instead of a remembered one.
        int[] tick = {0};
        int[] finishedAtTick = {-1};

        ScenarioRunner.run("ExampleAuto", ExampleRobotSim.ROBOT_CONFIG, timer, TICKS,
                DELTA_TIME_SECONDS, rlogPath, deltaTime -> {
                    tick[0]++;
                    elapsedInStep[0] += deltaTime;

                    switch (step[0]) {
                        case DRIVING:
                            sim.robot.drive.drive(DRIVE_POWER, 0.0, 0.0);
                            if (elapsedInStep[0] >= DRIVE_SECONDS) {
                                sim.robot.drive.stop();
                                sim.robot.slide.setLevel(ExampleSlide.Level.SCORE);
                                step[0] = Step.LIFTING;
                                elapsedInStep[0] = 0.0;
                            }
                            break;

                        case LIFTING:
                            if (sim.robot.slide.isAtTarget()) {
                                sim.robot.claw.setState(ExampleClaw.State.OPEN);
                                step[0] = Step.RELEASING;
                                elapsedInStep[0] = 0.0;
                            }
                            break;

                        case RELEASING:
                        case FINISHED:
                        default:
                            // RELEASING ends below, after this tick's physics - see the comment there.
                            break;
                    }

                    // The subsystems that close a loop run every tick regardless of which step it is.
                    sim.robot.periodic();
                    sim.advancePhysics(deltaTime);

                    // The moment the routine believes the jaws are open. Everything it would do next -
                    // lowering, driving away - depends on that belief, so this is where it gets checked.
                    //
                    // AFTER advancePhysics, not inside the switch above, and the difference is one tick.
                    // The switch runs before this tick's physics, so reading the jaws there would sample
                    // them 0.02 s short of the wait the assertion claims to be testing - a test that is
                    // very slightly stricter than the thing it says it is measuring.
                    if (step[0] == Step.RELEASING && elapsedInStep[0] >= ExampleClaw.TRAVEL_TIME_SECONDS) {
                        jawsAtFinish[0] = sim.clawPlant.getPosition();
                        finishedAtTick[0] = tick[0];
                        step[0] = Step.FINISHED;
                    }

                    Logger.recordOutput("Auto/step", step[0].name());
                    // The pose AdvantageScope's 3D field reads. The separate scalars below stay so the
                    // same run can also be plotted on ordinary line graphs.
                    sim.logPose("View/robotPose");
                    Logger.recordOutput("Auto/xInches", sim.drivePlant.getX());
                    Logger.recordOutput("Auto/yInches", sim.drivePlant.getY());
                    Logger.recordOutput("Auto/headingRadians", sim.drivePlant.getHeading());
                    Logger.recordOutput("Slide/heightInches", sim.robot.slide.getPositionInches());
                    Logger.recordOutput("Claw/commandedPosition", sim.robot.claw.getCommandedPosition());
                    Logger.recordOutput("Claw/jawsPosition", sim.clawPlant.getPosition());
                });

        assertTrue(step[0] == Step.FINISHED,
                "the routine never finished - it got as far as " + step[0]);

        // Finished with the budget to spare that TICKS describes. This is what stops that comment from
        // slowly becoming untrue: a change that makes the routine take half again as long fails here,
        // while the scenario still had ticks left and could say so, rather than failing later with
        // "never finished" and no clue which step was slow.
        assertTrue(finishedAtTick[0] < TICKS * 3 / 4,
                "the routine finished on tick " + finishedAtTick[0] + " of " + TICKS
                        + ", which is later than the tick budget was sized for");

        // Drove forward. Bounds are loose on purpose; see the class comment.
        assertTrue(sim.drivePlant.getX() > 5.0,
                "the robot barely moved forward: " + sim.drivePlant.getX() + " inches");
        assertTrue(sim.drivePlant.getX() < 60.0,
                "the robot drove further than the field is deep: " + sim.drivePlant.getX() + " inches");

        // Went straight. This is the assertion that catches a reversed or mis-signed drive motor, and it
        // is tight because a chassis told to go straight has no business going anywhere else.
        assertTrue(Math.abs(sim.drivePlant.getY()) < 1.0,
                "the robot drifted sideways while driving straight: " + sim.drivePlant.getY() + " inches");
        assertTrue(Math.abs(sim.drivePlant.getHeading()) < 0.05,
                "the robot turned while driving straight: " + sim.drivePlant.getHeading() + " radians");

        // The jaws really were open when the routine let go.
        double commandedOpen = ExampleClaw.State.OPEN.command();
        assertTrue(Math.abs(jawsAtFinish[0] - commandedOpen) < 0.02,
                "the routine released after waiting " + ExampleClaw.TRAVEL_TIME_SECONDS
                        + " s, but the jaws were still at " + jawsAtFinish[0]
                        + " rather than " + commandedOpen + " - the wait is too short");
    }
}
