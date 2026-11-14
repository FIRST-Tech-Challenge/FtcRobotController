package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Turret Subsystem
 * Owns the motor that turns the shooter hood left and right. Only this class touches
 * the turret motor, the turret encoder, and the home switch. OpModes call its methods.
 *
 * ANGLE CONVENTION (looking down from above, the robot front / intake is at the top):
 *
 *                      0 deg   <- the HOME SWITCH, pointing at the intake
 *             +90 deg (left)     -90 deg (right)
 *               dead zone behind the robot: about +/-135 to 180 deg
 *
 *   - Angles are SIGNED. Left (counterclockwise) is POSITIVE. Right (clockwise) is NEGATIVE.
 *   - That matches how Pinpoint and Pedro Pathing measure the robot's heading, so later:
 *         shooter's field heading = robot heading + turret angle
 *   - POSITIVE motor power must turn the turret toward POSITIVE angles (left).
 *
 * HOW THE TURRET KNOWS WHERE IT IS
 *   FOR NOW the turret motor's built-in encoder counts the turret's movement (the motor's
 *   16T pinion meshes with the turret's ring gear). Later, a REV Through Bore Encoder on its
 *   own pinion can replace it (see ENCODER_TICKS_PER_REV below).
 *   An encoder only counts movement, so it needs a reference. The REV magnetic limit switch is
 *   that reference:
 *     1. Before the match, place the turret by hand ON the switch (0 degrees).
 *        If it starts on the switch, we zero right away (see setUpHardware).
 *     2. During the match, every time the turret rolls onto the switch, update() resets
 *        the angle to 0 so any drift is corrected. To stop this from firing over and
 *        over while the turret hovers at the magnet's edge, it only re-arms after the
 *        turret has moved REARM_DISTANCE_DEGREES away from home.
 *   We never reset the encoder just because an OpMode starts. The Control Hub keeps its
 *   counts between Auto and TeleOp, so the turret keeps its angle when TeleOp begins.
 *   isZeroConfirmed() is false until the switch has been seen at least once.
 *   Nothing here moves the turret at INIT, because the rules say the robot must not move
 *   after initialization (G304, G403).
 *
 * TWO WAYS TO MOVE THE TURRET
 *   1. POSITION: setTargetAngle(degrees). A small controller turns the turret there.
 *   2. MANUAL: setManualPower(power). The driver steers it with a stick.
 *   Either way the turret will not be driven past its soft limits (+/-120 degrees,
 *   see SOFT_LIMITS_ON).
 *
 * STATE (what we told it to do) is IDLE, MANUAL, or POSITION.
 * STATUS (what the sensors say) is getAngleDegrees(), isHomeSwitchActive(), isOnTarget().
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name:           turret_motor        (goBILDA 5203, 435 RPM, power on the port's power
 *                                              connector. FOR NOW its own encoder cable is plugged
 *                                              into the SAME port's 4-pin ENCODER connector.
 *                                              Later the REV Through Bore Encoder can go in that
 *                                              connector instead. One config entry either way.)
 *   HOME SWITCH IS OFF FOR NOW (not installed, code commented out). Without it, 0 degrees is
 *   wherever the turret was when the Control Hub powered on, so start with the turret
 *   straight ahead, and the angle is never re-zeroed during a match.
 *   When installed: Digital Device name: turret_home_sensor (REV magnetic limit switch. Configure it as a
 *                                              "Digital Device", NOT "REV Touch Sensor",
 *                                              because this class reads it as a DigitalChannel.)
 */
public class TurretSubsystem {

    // ------------------------------------------------------------------
    // Hardware names. These MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String TURRET_MOTOR_NAME = "turret_motor";
    // HOME SWITCH IS NOT INSTALLED YET, so it is commented out here and in the other places
    // marked "HOME SWITCH OFF". To turn it back on, remove the // from each of those lines
    // and delete the "return false;" line in isHomeSwitchActive().
    // HOME SWITCH OFF: private static final String HOME_SWITCH_NAME = "turret_home_sensor";

    // ------------------------------------------------------------------
    // Directions. These two MUST be right or the turret will run away!
    // ------------------------------------------------------------------
    // Positive power must turn the turret LEFT (counterclockwise, toward positive angles).
    // If positive power turns it right, flip this between FORWARD and REVERSE.
    //
    // WATCH OUT: the Through Bore Encoder is plugged into this same motor port, so this
    // setting flips the encoder count too. If you flip it, check the angle direction again.
    private static final DcMotor.Direction TURRET_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;

    // Turning the turret LEFT must make the angle go UP.
    // Because the encoder shares the motor's port, we cannot fix its sign with setDirection()
    // without also reversing the motor. If the angle goes DOWN when you turn left (and the
    // motor direction above is already right), flip this between false and true.
    private static final boolean INVERT_ENCODER_READING = false;

    // ------------------------------------------------------------------
    // Gear and encoder numbers: how encoder ticks turn into turret degrees
    // ------------------------------------------------------------------
    // FOR NOW we use the turret motor's BUILT-IN encoder (the goBILDA 5203, 435 RPM, 13.7:1
    // gearbox). goBILDA lists it at 384.5 ticks per turn of the motor's output shaft.
    // goBILDA's spec sheet for this motor (5203-2402-0014 in TeamCode/docs/) says 28 pulses per
    // rev at the encoder shaft and 384.5 at the gearbox output shaft (28 x 13.7). The hand test
    // below checks the whole chain (this number and the gear teeth): if the angle is off, look
    // at the gear teeth first.
    //
    // TO GO BACK to the REV Through Bore Encoder: use 8192.0 here (2048 cycles x 4 edges), and
    // ENCODER_PINION_TEETH stays 16 because the encoder pinion is also a 16T gear.
    private static final double ENCODER_TICKS_PER_REV = 384.5;

    // The gear on the encoder's shaft and the ring gear mesh, so the encoder turns more than
    // the turret does. Here that gear is the MOTOR'S 16T pinion, because the motor's encoder
    // is on the motor shaft: 3 motor turns make 1 turret turn.
    // The ring gear has 47 teeth (the coach checked it, 2026-11-12). Both pinions are 16.
    private static final double RING_GEAR_TEETH = 47.0;
    private static final double ENCODER_PINION_TEETH = 16.0;

    private static final double DEGREES_PER_REV = 360.0;

    // Encoder ticks for ONE degree of turret turn. With the motor encoder and 47 / 16 teeth this
    // is about 3.14. (The Through Bore Encoder would give about 66.8.)
    // Check it on the robot: turn the turret by hand through a marked 90 degrees and see
    // that the angle on screen reads 90. If not, check the tooth counts above.
    private static final double TICKS_PER_TURRET_DEGREE =
            ENCODER_TICKS_PER_REV * (RING_GEAR_TEETH / ENCODER_PINION_TEETH) / DEGREES_PER_REV;

    // ------------------------------------------------------------------
    // Home switch
    // ------------------------------------------------------------------
    // The angle of the home switch. 0 means the switch is at the turret's center (pointing
    // at the intake). If you ever move the switch, change this.
    private static final double HOME_ANGLE_DEGREES = 0.0;

    // The REV magnetic limit switch is ACTIVE-LOW: it reads false when a magnet is near.
    // So "active" (turret is on the switch) means getState() == false.
    // If the Home Switch line in telemetry looks backwards, flip this to true.
    private static final boolean HOME_SWITCH_ACTIVE_STATE = false;

    // After a re-zero, the turret must move this far from home before the next re-zero.
    // That stops the angle from jumping around while the turret sits at the magnet's edge.
    private static final double REARM_DISTANCE_DEGREES = 10.0;

    // ------------------------------------------------------------------
    // Soft limits: the turret will not be driven past these angles.
    // ------------------------------------------------------------------
    // The full travel is about 270 degrees, which is +/-135. These sit 15 degrees inside that,
    // at +/-120, so there is room for the turret to coast a little past the limit before it
    // could reach the cables or the frame.
    //
    // THE LIMITS ARE ONLY AS GOOD AS THE ANGLE. Without the home switch, 0 degrees is wherever
    // the turret pointed when the Control Hub powered on. Power on with the turret pointing
    // straight ahead, or +/-120 will be measured from the wrong place.
    //
    // To turn the limits off for a test, set SOFT_LIMITS_ON to false. Then nothing stops the
    // turret at an angle, so keep a hand on the Stop button.
    private static final boolean SOFT_LIMITS_ON = true;
    private static final double MIN_ANGLE_DEGREES = -120.0;
    private static final double MAX_ANGLE_DEGREES = 120.0;

    // ------------------------------------------------------------------
    // Position control settings. All of these are starting guesses to tune on the robot.
    // ------------------------------------------------------------------
    // P: motor power per degree of error. 0.02 means 30 degrees of error asks for 60% power.
    // Raise it if the turret is slow to get there. Lower it if it overshoots or shakes.
    private static final double KP = 0.02;

    // D: slows the push as the turret speeds up, to calm overshoot.
    // Leave at 0.0 until the turret overshoots or oscillates. Then try a small number like 0.001.
    private static final double KD = 0.0;

    // A geared turret sticks a little. This extra power gets it started moving.
    // If the turret stops short of the target, raise it. If it jerks, lower it.
    private static final double FRICTION_POWER = 0.05;

    // The most power position control will use (auto-align and the presets). It was 0.5 for the
    // first tests, then 0.75, and is 1.0 (full power) now. 1.0 is the most this can be.
    // A faster turret overshoots more, and the stopping distance grows with the SQUARE of the
    // speed (twice as fast takes about four times as far to stop). If it swings past the target,
    // lower this, or try a small KD. Also re-check how far it coasts past the +/-120 degree limits.
    private static final double MAX_POWER = 1.0;

    // The most power MANUAL mode can use. setManualPower(1.0) means this much. It is 1.0 now
    // because the test OpMode (TurretAlpha) lets the operator pick the speed with a dial.
    private static final double MAX_MANUAL_POWER = 1.0;

    // The turret counts as "on target" when it is within this many degrees of the target...
    // The hive window forgives about +/-5 to 13 degrees, so this can be fairly loose.
    private static final double ON_TARGET_TOLERANCE_DEGREES = 1.5;

    // ...for at least this long. This stops the answer flashing while it passes the target.
    private static final double ON_TARGET_HOLD_SECONDS = 0.1;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // STATE = what we told the turret to do.
    public enum TurretState {
        IDLE,      // Not driving the turret. It is free to turn by hand.
        MANUAL,    // Running at a power the driver asked for.
        POSITION   // Turning to, and holding, a target angle.
    }

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    // The motor object also reads the Through Bore Encoder, which shares its port.
    private DcMotorEx turretMotor;
    // HOME SWITCH OFF: private DigitalChannel homeSwitch;

    private TurretState state = TurretState.IDLE;

    private double targetAngleDegrees = 0.0;
    private double manualPower = 0.0;

    // An extra push added on top of position control, so the turret can follow a robot that is
    // spinning (see setFeedforwardPower). It lasts ONE loop: update() sets it back to 0.0 after
    // using it, so an OpMode that wants it must send it again every loop.
    private double feedforwardPower = 0.0;

    // What the sensors reported the last time update() ran.
    private int encoderTicks = 0;
    private double angleDegrees = 0.0;
    private double previousAngleDegrees = 0.0;
    private double angularVelocityDegreesPerSecond = 0.0;

    // Home switch bookkeeping.
    private boolean homeSwitchWasActive = false;
    private boolean zeroConfirmed = false;
    private boolean rearmed = true;

    // For telemetry: the angle the turret THOUGHT it was at when it reached or left the switch.
    // When it reaches the switch, this shows how far off (drift) the angle was before the re-zero.
    // These two numbers also show whether the switch triggers at a different angle
    // depending on which way the turret is moving.
    private double angleWhenReachedSwitch = 0.0;
    private double angleWhenLeftSwitch = 0.0;

    // "On target" bookkeeping.
    private ElapsedTime onTargetTimer = new ElapsedTime();
    private boolean wasWithinTolerance = false;
    private boolean onTarget = false;

    // Times each loop, so we can work out how fast the turret is turning.
    private ElapsedTime loopTimer = new ElapsedTime();

    // True once we have asked the motor to hold position with BRAKE.
    private boolean brakeOn = false;

    /*
     * Setup. Gets the motor and the home switch from the robot configuration.
     * (The encoder comes along with the motor, because it is plugged into the motor's port.)
     * The turret starts in IDLE, free to turn by hand.
     */
    public TurretSubsystem(HardwareMap hardwareMap) {
        // The names MUST match the robot configuration exactly.
        turretMotor = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
        // HOME SWITCH OFF: homeSwitch = hardwareMap.get(DigitalChannel.class, HOME_SWITCH_NAME);

        setUpHardware();
    }

    /*
     * Sets directions and modes, then works out the starting angle.
     */
    private void setUpHardware() {
        turretMotor.setDirection(TURRET_MOTOR_DIRECTION);

        // HOME SWITCH OFF: homeSwitch.setMode(DigitalChannel.Mode.INPUT);

        // We control the motor with raw power (RUN_WITHOUT_ENCODER means "no speed control",
        // the hub still counts encoder ticks). The count we read is from the Through Bore
        // Encoder on the turret ring gear, because that is what is plugged into this port.
        // NOTE: we do NOT reset the count here. The Control Hub keeps its count between Auto
        // and TeleOp, so the turret keeps its angle when the next OpMode starts.
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0.0);

        // FLOAT until someone commands the turret, so it can be turned by hand during INIT
        // to sit on the switch. Once we command it, we switch to BRAKE (see useBrake).
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brakeOn = false;

        // If the turret was placed on the switch, we know exactly where it is: 0 degrees.
        homeSwitchWasActive = isHomeSwitchActive();
        if (homeSwitchWasActive) {
            zeroAtHome();
        }

        encoderTicks = readEncoderTicks();
        angleDegrees = ticksToAngleDegrees(encoderTicks);
        previousAngleDegrees = angleDegrees;
        loopTimer.reset();
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the turret what to do
    // ==================================================================

    /*
     * Turn the turret to an angle and hold it there. Positive is LEFT, negative is RIGHT,
     * 0 is straight ahead (the intake direction). The angle is held inside the soft limits
     * (when SOFT_LIMITS_ON is true).
     *
     * Safe to call every loop with the same angle. A repeat call changes nothing.
     */
    public void setTargetAngle(double degrees) {
        double newTargetAngle = degrees;
        if (SOFT_LIMITS_ON) {
            newTargetAngle = Range.clip(degrees, MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
        }

        // Same request as last time? Then there is nothing to do.
        if (state == TurretState.POSITION && newTargetAngle == targetAngleDegrees) {
            return;
        }

        useBrake();
        targetAngleDegrees = newTargetAngle;
        wasWithinTolerance = false;
        onTarget = false;
        state = TurretState.POSITION;
    }

    /*
     * Hold the turret wherever it is right now. Use this when the driver lets go of the stick.
     */
    public void holdCurrentAngle() {
        setTargetAngle(angleDegrees);
    }

    /*
     * Add an extra push to position control so the turret FOLLOWS the robot as it spins.
     * Use -1.0 to 1.0. Positive pushes the turret LEFT, negative pushes it RIGHT, the same as
     * setManualPower(). When the robot spins left (counterclockwise), the turret has to turn
     * RIGHT to keep pointing at the hive, so the OpMode sends a NEGATIVE number.
     *
     * WHY: position control only pushes when the turret is already off target, so on its own
     * it always trails a spinning robot by some degrees. This push starts the turret moving
     * before it falls behind, and position control only has to clean up what is left.
     *
     * It only matters in POSITION mode, and it lasts ONE loop. Call it every loop, before
     * update(). The max power and the soft limits still apply to the total.
     */
    public void setFeedforwardPower(double power) {
        feedforwardPower = Range.clip(power, -1.0, 1.0);
    }

    /*
     * Drive the turret at a power. Use -1.0 to 1.0, where 1.0 means MAX_MANUAL_POWER.
     * Positive turns the turret LEFT (toward positive angles). Negative turns it RIGHT.
     * The soft limits still stop it (when SOFT_LIMITS_ON is true).
     */
    public void setManualPower(double power) {
        useBrake();
        manualPower = Range.clip(power, -1.0, 1.0) * MAX_MANUAL_POWER;
        wasWithinTolerance = false;
        onTarget = false;
        state = TurretState.MANUAL;
    }

    /*
     * Stop driving the turret. It floats, so it can be turned by hand. Safe to call every loop.
     */
    public void stop() {
        turretMotor.setPower(0.0);
        useFloat();
        manualPower = 0.0;
        wasWithinTolerance = false;
        onTarget = false;
        state = TurretState.IDLE;
    }

    /*
     * Call this ONCE per loop(), after you give the turret its commands.
     * It reads the encoder and the home switch, fixes the angle if the turret just reached
     * the switch, and then drives the motor.
     */
    public void update() {
        // 1. Read the encoder and work out the angle.
        encoderTicks = readEncoderTicks();
        angleDegrees = ticksToAngleDegrees(encoderTicks);
        double secondsSinceLastLoop = loopTimer.seconds();
        loopTimer.reset();

        // 2. Check the home switch.
        boolean didRezero = updateHomeSwitch();

        // 3. How fast is the turret turning? (A re-zero makes the angle jump, so skip that loop.)
        if (didRezero || secondsSinceLastLoop <= 0.0) {
            angularVelocityDegreesPerSecond = 0.0;
        } else {
            angularVelocityDegreesPerSecond =
                    (angleDegrees - previousAngleDegrees) / secondsSinceLastLoop;
        }
        previousAngleDegrees = angleDegrees;

        // 4. Drive the motor. IDLE means we leave the motor alone.
        if (state == TurretState.MANUAL) {
            turretMotor.setPower(applySoftLimits(manualPower));
        } else if (state == TurretState.POSITION) {
            turretMotor.setPower(applySoftLimits(calculatePositionPower()));
        }

        // The feedforward lasts one loop. Clearing it here means a push that nobody renewed can
        // never keep the turret moving.
        feedforwardPower = 0.0;

        // 5. Are we on target?
        updateOnTarget();
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the turret what is happening
    // ==================================================================

    /*
     * True when the turret is in POSITION mode and has been within tolerance of its target
     * for a short time. This is the turret's part of the "OK to feed a ball" check.
     */
    public boolean isOnTarget() {
        return onTarget;
    }

    // True once the turret has been on the home switch at least once, so the angle is trusted.
    public boolean isZeroConfirmed() {
        return zeroConfirmed;
    }

    // True while the turret is sitting on the home switch (a magnet is near the sensor).
    public boolean isHomeSwitchActive() {
        // HOME SWITCH OFF: with no switch installed, the turret is never "on the switch".
        // That means the angle is never re-zeroed and isZeroConfirmed() stays false.
        return false;
        // HOME SWITCH OFF: return homeSwitch.getState() == HOME_SWITCH_ACTIVE_STATE;
    }

    public TurretState getState() {
        return state;
    }

    // The turret's angle in degrees as of the last update(). Positive is left.
    public double getAngleDegrees() {
        return angleDegrees;
    }

    /*
     * The soft limits, in degrees. The OpMode can use them to tell whether an angle is somewhere
     * the turret can go, so it never has to copy these numbers.
     */
    public double getMinAngleDegrees() {
        return MIN_ANGLE_DEGREES;
    }

    public double getMaxAngleDegrees() {
        return MAX_ANGLE_DEGREES;
    }

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }

    // The raw encoder count. Use this to check the ticks per degree by hand.
    public int getEncoderTicks() {
        return encoderTicks;
    }

    /*
     * Adds the turret's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Turret State", state);
        telemetry.addData("Angle", "%.1f deg", angleDegrees);
        telemetry.addData("Target", "%.1f deg   (error %.1f)",
                targetAngleDegrees, targetAngleDegrees - angleDegrees);
        telemetry.addData("On Target?", onTarget);
        telemetry.addData("Zero Confirmed?", zeroConfirmed);
        // HOME SWITCH OFF: the real lines are below. Put them back when the switch is installed.
        telemetry.addData("Home Switch", "NOT INSTALLED (0 deg = where the turret was at power-up)");
        // telemetry.addData("Home Switch", "%s   (raw state = %b)",
        //         isHomeSwitchActive() ? "ON the switch" : "off the switch", homeSwitch.getState());
        telemetry.addData("Encoder ticks", encoderTicks);
        if (SOFT_LIMITS_ON) {
            telemetry.addData("Soft limits", "%.0f to %.0f deg", MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
        } else {
            telemetry.addData("Soft limits", "OFF (nothing stops the turret!)");
        }
        telemetry.addData("Angle reached switch at", "%.1f deg  (before re-zero)", angleWhenReachedSwitch);
        telemetry.addData("Angle left switch at", "%.1f deg", angleWhenLeftSwitch);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    /*
     * Looks at the home switch. When the turret rolls onto the switch (and has re-armed),
     * the angle is reset to the home angle. Returns true if that just happened.
     */
    private boolean updateHomeSwitch() {
        boolean switchActive = isHomeSwitchActive();
        boolean justReachedSwitch = switchActive && !homeSwitchWasActive;
        boolean justLeftSwitch = !switchActive && homeSwitchWasActive;
        homeSwitchWasActive = switchActive;

        if (justLeftSwitch) {
            angleWhenLeftSwitch = angleDegrees;
        }

        // Once the turret has moved far enough from home, the next arrival counts again.
        if (!rearmed && Math.abs(angleDegrees - HOME_ANGLE_DEGREES) >= REARM_DISTANCE_DEGREES) {
            rearmed = true;
        }

        if (justReachedSwitch) {
            angleWhenReachedSwitch = angleDegrees;
            if (rearmed) {
                zeroAtHome();
                angleDegrees = HOME_ANGLE_DEGREES;
                return true;
            }
        }
        return false;
    }

    /*
     * Reads the Through Bore Encoder (through the turret motor's port) and fixes its sign
     * if INVERT_ENCODER_READING says so.
     */
    private int readEncoderTicks() {
        int ticks = turretMotor.getCurrentPosition();
        if (INVERT_ENCODER_READING) {
            return -ticks;
        }
        return ticks;
    }

    /*
     * Tells the encoder "this spot is the home angle". We reset its count to zero, and the
     * angle is worked out from the count from then on.
     * We use 0 ticks = HOME_ANGLE_DEGREES.
     *
     * The reset also makes the motor stop for an instant, because it is the same port. That is
     * fine: update() sends the motor power again later in the same loop.
     */
    private void zeroAtHome() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        zeroConfirmed = true;
        rearmed = false;
    }

    /*
     * The position controller. Returns the motor power that moves the turret toward the target.
     */
    private double calculatePositionPower() {
        double error = targetAngleDegrees - angleDegrees;

        // Close enough: stop pushing and let the BRAKE hold the turret. The feedforward still
        // goes through (it is 0.0 unless the OpMode sent one this loop), so a turret that is
        // following a spinning robot keeps following instead of stopping inside this band.
        if (Math.abs(error) <= ON_TARGET_TOLERANCE_DEGREES) {
            return Range.clip(feedforwardPower, -MAX_POWER, MAX_POWER);
        }

        // P pushes harder the further away we are. D slows the push when we are already
        // turning fast. Friction power gets the turret moving.
        // (If you turn KD on while using the feedforward, KD will push AGAINST the turret's
        // following motion, so lower the feedforward gain to match.)
        double power = KP * error - KD * angularVelocityDegreesPerSecond;
        if (error > 0.0) {
            power = power + FRICTION_POWER;
        } else {
            power = power - FRICTION_POWER;
        }

        // The feedforward goes on top of all that.
        power = power + feedforwardPower;
        return Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    /*
     * Never push the turret past a soft limit. At the limit, power that pushes further is
     * cut to zero. Power that pulls back toward the middle is still allowed.
     */
    private double applySoftLimits(double power) {
        // The soft limits can be switched off (see SOFT_LIMITS_ON).
        if (!SOFT_LIMITS_ON) {
            return power;
        }
        if (angleDegrees >= MAX_ANGLE_DEGREES && power > 0.0) {
            return 0.0;
        }
        if (angleDegrees <= MIN_ANGLE_DEGREES && power < 0.0) {
            return 0.0;
        }
        return power;
    }

    /*
     * Works out whether we have been within tolerance of the target long enough.
     */
    private void updateOnTarget() {
        if (state != TurretState.POSITION) {
            wasWithinTolerance = false;
            onTarget = false;
            return;
        }

        boolean withinTolerance =
                Math.abs(targetAngleDegrees - angleDegrees) <= ON_TARGET_TOLERANCE_DEGREES;
        if (!withinTolerance) {
            wasWithinTolerance = false;
            onTarget = false;
            return;
        }

        // Within tolerance. Start the timer the first time, then wait out the hold time.
        if (!wasWithinTolerance) {
            onTargetTimer.reset();
            wasWithinTolerance = true;
        }
        if (onTargetTimer.seconds() >= ON_TARGET_HOLD_SECONDS) {
            onTarget = true;
        }
    }

    // Turns an encoder count into a turret angle. 0 ticks is the home angle.
    private double ticksToAngleDegrees(int ticks) {
        return HOME_ANGLE_DEGREES + ticks / TICKS_PER_TURRET_DEGREE;
    }

    // BRAKE holds the turret in place when power is 0. We only turn it on once the turret
    // is being commanded, so it is easy to turn by hand during INIT.
    private void useBrake() {
        if (brakeOn) {
            return;
        }
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brakeOn = true;
    }

    private void useFloat() {
        if (!brakeOn) {
            return;
        }
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brakeOn = false;
    }
}
