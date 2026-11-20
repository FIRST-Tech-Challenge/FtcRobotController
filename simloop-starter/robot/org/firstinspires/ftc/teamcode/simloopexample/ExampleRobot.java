package org.firstinspires.ftc.teamcode.simloopexample;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The invented robot the SimLoop example drives: a mecanum chassis, one vertical linear slide, and one
 * claw on a servo.
 *
 * <p>This class is the pattern worth copying, more than any single subsystem is. <b>One object owns the
 * hardware map and constructs every subsystem from it</b>, so there is exactly one place that decides
 * what a robot is made of. An OpMode builds one of these; a simulated test builds one of these; neither
 * knows how the other got its hardware map, because both hand it the same constructor argument.
 *
 * <p>Without that, a simulated test has to construct each subsystem itself, and the moment a subsystem
 * is added to the robot and not to the test, the test is quietly running a different robot than the one
 * that drives onto the field.
 *
 * <p><b>This is not any real team's competition robot.</b> It is the simplest machine that still has
 * all three kinds of actuator a season robot has - a drivetrain, a position-controlled mechanism, and a
 * servo - because those are the three things a team needs to see simulated before trusting the rest.
 */
public class ExampleRobot {

    /** The mecanum chassis. */
    public final ExampleDrive drive;

    /** The vertical linear slide. */
    public final ExampleSlide slide;

    /** The claw at the top of the slide. */
    public final ExampleClaw claw;

    /**
     * Builds every subsystem from one hardware map.
     *
     * @param hardwareMap the OpMode's hardware map on a robot, or a {@code FakeHardwareMap} in a
     *                    simulated test. This constructor is the only place either one is named, which
     *                    is what lets the same robot run in both.
     */
    public ExampleRobot(HardwareMap hardwareMap) {
        this.drive = new ExampleDrive(hardwareMap);
        this.slide = new ExampleSlide(hardwareMap);
        this.claw = new ExampleClaw(hardwareMap);
    }

    /**
     * Runs one step of every subsystem that closes a loop. Call this once per OpMode loop, and once per
     * tick in a simulated scenario.
     *
     * <p>The drivetrain is absent on purpose: it is commanded, not closed-loop, so it has nothing to do
     * between commands.
     */
    public void periodic() {
        slide.periodic();
    }
}
