package org.firstinspires.ftc.teamcode.simloopexample;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * A vertical linear slide, written the way an ordinary FTC subsystem is written.
 *
 * <p><b>Nothing in this class knows it can be simulated.</b> It resolves its hardware from a
 * {@link HardwareMap}, reads an encoder, runs a proportional controller and commands a motor. That is
 * the point of the example: the class that runs in {@code ExampleSlideSimTest} on a laptop is this
 * class, unchanged, and not a test double standing in for it.
 *
 * <p><b>The robot this belongs to is invented.</b> Every number below is a plausible round figure for a
 * two-stage slide on a 435&nbsp;rpm motor, chosen so the example is easy to read. None of them is
 * measured off any competition robot, so do not treat them as a starting point for tuning - measure
 * yours.
 *
 * <h2>Why an external encoder rather than the motor's own</h2>
 *
 * <p>The subsystem closes its loop on a separate encoder that measures the carriage, not on the motor's
 * built-in one, and never on {@code RUN_TO_POSITION}. On hardware that is a through-bore encoder wired
 * to a spare motor-encoder port, which is why it is resolved as a {@link DcMotorEx} like any other. It
 * matters here because the two disagree in exactly the case you care about: when the slide is against an
 * end stop, the motor's shaft encoder keeps counting and the carriage encoder does not.
 */
public class ExampleSlide {

    /** Encoder counts per inch of carriage travel. Invented; measure yours. */
    public static final double TICKS_PER_INCH = 41.7;

    /** How close to the target counts as arrived, in inches. */
    public static final double TOLERANCE_INCHES = 0.5;

    /** Proportional gain, in motor power per inch of error. Invented; tune yours. */
    private static final double KP_POWER_PER_INCH = 0.25;

    /**
     * Constant power the carriage needs just to hold its own weight up, unitless in [0, 1). Invented;
     * measure yours by nudging the power up until a raised slide stops sinking.
     *
     * <p>This term is the reason a simulation that models gravity is worth more than one that does not.
     * A proportional controller with no feedforward parks where its own output equals the weight, so
     * without this the carriage would settle {@code weight / KP} short of target - about a third of an
     * inch here - and would do it on the robot whether or not anything in simulation ever showed it. A
     * plant with no gravity in it holds any height at zero power, so that controller would look perfect
     * on a laptop and sag on the field.
     */
    private static final double GRAVITY_FEEDFORWARD_POWER = 0.08;

    /** Lowest and highest the carriage may be commanded to, in inches. */
    private static final double MIN_TARGET_INCHES = 0.0;
    private static final double MAX_TARGET_INCHES = 24.0;

    /** The named heights this slide is driven to. Heights are in inches of carriage travel. */
    public enum Level {
        /** All the way down, where the claw can reach a floor sample. */
        FLOOR(0.0),
        /** Part way up, clear of the wall. */
        CARRY(6.0),
        /** The scoring height this example drives to. */
        SCORE(18.0);

        private final double heightInches;

        Level(double heightInches) {
            this.heightInches = heightInches;
        }

        /** {@return this level's carriage height, in inches above the bottom of travel} */
        public double heightInches() {
            return heightInches;
        }
    }

    /** Drives the carriage. Power in, no encoder read - see the class comment. */
    private final DcMotorEx motor;

    /** Measures the carriage. Read only; this subsystem never powers it. */
    private final DcMotorEx encoder;

    /** Where the carriage is being driven to, in inches above the bottom of travel. */
    private double targetInches = Level.FLOOR.heightInches();

    /**
     * Resolves the slide's two devices from the robot's configuration.
     *
     * @param hardwareMap the OpMode's hardware map on a robot, or a {@code FakeHardwareMap} in a
     *                    simulated test - this class cannot tell the difference and does not try
     */
    public ExampleSlide(HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        this.encoder = hardwareMap.get(DcMotorEx.class, "slideEncoder");
        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Sets the height the slide drives to.
     *
     * @param level the named height to drive to
     */
    public void setLevel(Level level) {
        setTargetInches(level.heightInches());
    }

    /**
     * Sets the height the slide drives to, clamped to the range the carriage can physically reach.
     *
     * @param inches the requested carriage height, in inches above the bottom of travel
     */
    public void setTargetInches(double inches) {
        this.targetInches = Math.max(MIN_TARGET_INCHES, Math.min(MAX_TARGET_INCHES, inches));
    }

    /** {@return the height the slide is currently driving to, in inches above the bottom of travel} */
    public double getTargetInches() {
        return targetInches;
    }

    /** {@return where the carriage actually is, in inches above the bottom of travel} */
    public double getPositionInches() {
        return encoder.getCurrentPosition() / TICKS_PER_INCH;
    }

    /** {@return the power last commanded to the slide motor, in [-1, 1]} */
    public double getCommandedPower() {
        return motor.getPower();
    }

    /** {@return whether the carriage is within {@link #TOLERANCE_INCHES} of its target} */
    public boolean isAtTarget() {
        return Math.abs(targetInches - getPositionInches()) <= TOLERANCE_INCHES;
    }

    /**
     * Runs one step of the controller. Call this once per loop, from the OpMode on a robot and from the
     * scenario body in a simulated test - the same once-per-loop contract in both places.
     */
    public void periodic() {
        double errorInches = targetInches - getPositionInches();
        double requested = errorInches * KP_POWER_PER_INCH + GRAVITY_FEEDFORWARD_POWER;
        motor.setPower(Math.max(-1.0, Math.min(1.0, requested)));
    }
}
