package org.firstinspires.ftc.teamcode.simloopexample;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A two-position claw on one servo, written the way an ordinary FTC subsystem is written.
 *
 * <p>It is here because a servo is the mechanism most teams reach for first and the one most likely to be
 * left out of a simulation, on the grounds that a servo just goes where it is told. It does not: it
 * takes time to get there.
 *
 * <p><b>A bare servo has no sensor</b>, so this class genuinely cannot tell you where the jaws are - and
 * neither can a real one. {@link #getCommandedPosition()} is the command, not a measurement, and there
 * is deliberately no {@code isSettled()} reading it back, because such a method would return true on the
 * tick the command was issued and would be a lie on hardware as well as in simulation.
 *
 * <p>What a routine does instead is wait {@link #TRAVEL_TIME_SECONDS} after commanding, and the right
 * question is whether that number is big enough. That is exactly the question simulation can answer:
 * SimLoop models the travel in a {@code PositionalServoPlant}, so the example's test drives this
 * subsystem, waits the constant below, and then asserts against the modelled jaws. Shorten the constant
 * and the test fails - which on the field would have been a sample dropped on the way up.
 *
 * <p><b>The robot this belongs to is invented</b>, and so are the two servo positions.
 */
public class ExampleClaw {

    /** Servo command that holds the jaws shut on a sample. Unitless, in [0, 1]. */
    private static final double CLOSED_COMMAND = 0.18;

    /** Servo command that opens the jaws wide enough to release. Unitless, in [0, 1]. */
    private static final double OPEN_COMMAND = 0.62;

    /**
     * How long a routine must wait after commanding the claw before the jaws have finished moving, in
     * seconds.
     *
     * <p>This is the number a team guesses and then never revisits. It is public because the example's
     * test reads it rather than repeating it: the test is an assertion <b>about this constant</b>, and a
     * test carrying its own copy would keep passing after someone shortened this one.
     */
    public static final double TRAVEL_TIME_SECONDS = 0.45;

    /** The claw's two states. */
    public enum State {
        /** Jaws shut, holding a sample. */
        CLOSED(CLOSED_COMMAND),
        /** Jaws open, ready to take or release. */
        OPEN(OPEN_COMMAND);

        private final double command;

        State(double command) {
            this.command = command;
        }

        /** {@return the servo command for this state, unitless in [0, 1]} */
        public double command() {
            return command;
        }
    }

    /** The servo that opens and shuts the jaws. */
    private final Servo servo;

    /** The state last commanded. Where the jaws actually are is not knowable from here. */
    private State state = State.CLOSED;

    /**
     * Resolves the claw servo from the robot's configuration.
     *
     * @param hardwareMap the OpMode's hardware map on a robot, or a {@code FakeHardwareMap} in a
     *                    simulated test
     */
    public ExampleClaw(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(Servo.class, "claw");
    }

    /**
     * Commands the claw to a state. Returns immediately, and the jaws are still moving when it does -
     * wait {@link #TRAVEL_TIME_SECONDS} before doing anything that assumes they arrived.
     *
     * @param state the state to drive to
     */
    public void setState(State state) {
        this.state = state;
        servo.setPosition(state.command());
    }

    /** {@return the state last commanded} */
    public State getState() {
        return state;
    }

    /**
     * {@return the servo command currently being held, unitless in [0, 1]}
     *
     * <p>This is what the claw is <b>asking for</b>. It is not a measurement of the jaws, and no method
     * on this class is - see the class comment.
     */
    public double getCommandedPosition() {
        return servo.getPosition();
    }
}
