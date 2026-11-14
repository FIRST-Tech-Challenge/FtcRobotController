package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Shooter Subsystem
 * Owns the shooter FLYWHEEL (goBILDA 5203 series, 1620 RPM, the 3.7:1 gearbox) and the HOOD
 * WHEEL (goBILDA 5203 series, 6000 RPM, 1:1).
 * Only this class touches those two motors. OpModes call its methods.
 *
 * It has two ways to run the wheels:
 *   1. SPEED CONTROL (the normal way): setTargetRpm() / setTargetRpms().
 *      You say how fast you want each wheel in RPM, and the motor controller
 *      works to hold that speed, even as the battery drains or a ball slows the wheel.
 *   2. OPEN LOOP: setPower(). You say how much power, and the speed is whatever
 *      it ends up being. Use this for tests and for clearing a jam (negative power).
 *
 * STATE (what we told the shooter to do) is one of: IDLE, OPEN_LOOP, SPINNING_UP, READY.
 * STATUS (what the encoders report) is read with getFlywheelRpm() and friends.
 *
 * The OpMode must call update() ONCE per loop(). Nothing calls it for us.
 *
 * "RPM" in this class is the speed of the motor's OUTPUT SHAFT, worked out from the encoder.
 * (For the 1620 RPM flywheel motor that is the shaft AFTER the 3.7:1 gearbox, which is why it
 * has more encoder ticks per turn than the hood wheel motor.)
 * TODO (issue #38): The flywheel belt is 20T to 18T, NOT 1:1, so the flywheel itself turns a
 * little faster or slower than this number. Which pulley is on the motor, and the hood wheel's
 * pulleys, still need checking. The numbers still work because we tune the shot RPMs by
 * measuring on the robot.
 *
 * Robot configuration (on the Driver Station):
 *   Motor name: flywheel   (goBILDA 5203 series, 1620 RPM 3.7:1, encoder plugged in)
 *   Motor name: hood_wheel   (goBILDA 5203 series, 6000 RPM, encoder plugged in)
 * Set both motors to the matching goBILDA motor type in the configuration. The
 * Control Hub's built-in speed-control numbers depend on the motor type.
 */
public class ShooterSubsystem {

    // ------------------------------------------------------------------
    // Hardware names. These MUST match the robot configuration exactly.
    // ------------------------------------------------------------------
    private static final String FLYWHEEL_MOTOR_NAME = "flywheel";
    private static final String HOOD_WHEEL_MOTOR_NAME = "hood_wheel";

    // Each motor has its OWN direction. These are copied from ShooterPowerAlpha.
    // We want a POSITIVE RPM to mean "shooting". If a wheel spins the wrong way,
    // flip that motor's line between FORWARD and REVERSE.
    private static final DcMotor.Direction FLYWHEEL_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction HOOD_WHEEL_DIRECTION = DcMotor.Direction.FORWARD;

    // ------------------------------------------------------------------
    // Encoder and motor numbers
    // ------------------------------------------------------------------
    // Encoder ticks per ONE turn of the motor's OUTPUT shaft. The two motors are different now.
    //
    // Both numbers come from the goBILDA spec sheets in TeamCode/docs/.
    //
    // HOOD WHEEL (6000 RPM, 1:1): 28. Spec sheet 5203-2402-0001: 28 pulses per rev at both the
    // encoder shaft and the output shaft. (Last season's notes said 112. That was wrong.)
    //
    // FLYWHEEL (1620 RPM, 3.7:1 gearbox): 103.8. Spec sheet 5203-2402-0003: 28 pulses per rev at
    // the encoder shaft and 103.8 at the gearbox output shaft (28 x the 3.7:1 gearbox).
    //
    // SANITY CHECK on the robot (the numbers are not in doubt, this just catches a wiring or
    // config mistake): the encoder sits on the motor shaft, BEFORE the gearbox, so both motors
    // read about 2,800 ticks/sec at full power. With the wheels clear, hold the RIGHT TRIGGER in
    // ShooterRPMAlpha. The hood wheel should read about 6,000 RPM and the flywheel about 1,620
    // RPM (a motor is a little under its top speed). If one reads far off, check the cables and
    // the motor type in the Driver Station config first, then ask the coach.
    private static final double FLYWHEEL_TICKS_PER_REV = 103.8;
    private static final double HOOD_WHEEL_TICKS_PER_REV = 28.0;

    // Each motor's top speed with no load. (goBILDA 5203 1620 RPM and 6000 RPM motors.)
    private static final double FLYWHEEL_MOTOR_MAX_RPM = 1620.0;
    private static final double HOOD_WHEEL_MOTOR_MAX_RPM = 6000.0;

    // The highest RPM we will let an OpMode ask for, for EACH wheel. Under load a motor cannot
    // hold its full no-load speed, so asking for 100% leaves the controller no room to catch up.
    // Both are 90% of the motor's top speed. Raise or lower them after you see what the wheels
    // can really hold. The flywheel motor is the slow one now, so it hits its limit first:
    // 1458 RPM, and the shot table's farthest spot (1425 RPM) is already close to it.
    private static final double FLYWHEEL_MAX_TARGET_RPM = 1458.0;
    private static final double HOOD_WHEEL_MAX_TARGET_RPM = 5400.0;

    private static final double SECONDS_PER_MINUTE = 60.0;

    // ------------------------------------------------------------------
    // Shooting settings
    // ------------------------------------------------------------------
    // HOOD_WHEEL_RATIO = HOOD WHEEL RPM / FLYWHEEL RPM.
    // When you use setTargetRpm(), the hood wheel's target = flywheel target x this number.
    // (setTargetRpms() ignores this and takes both speeds.)
    //
    // WHAT THE NUMBER DOES TO THE BALL
    //   Ball spin comes from the DIFFERENCE in how fast the two wheel SURFACES move, and
    //   surface speed = RPM x wheel diameter. Our flywheel is on the front and about 4 inches.
    //   Our hood wheel is on the back/top side, in the fixed hood, and about 2 inches.
    //     1.0         -> same RPM, so the flywheel surface is about TWICE as fast: BACKSPIN.
    //     about 2.0   -> both surfaces move the same speed: about NO spin.
    //     more than 2 -> the hood wheel surface is faster: TOPSPIN.
    //   Our fixed hood may rub the ball and add a little backspin, so the best number for no
    //   spin could be a bit above 2.0. Test 2.0, then try a little higher or lower.
    //
    // WATCH THE SPEED LIMIT
    //   Each wheel has its own cap (FLYWHEEL_MAX_TARGET_RPM 1458, HOOD_WHEEL_MAX_TARGET_RPM 5400).
    //   At a ratio of 2.0 the hood wheel only needs twice the flywheel's RPM, so at the top of the
    //   flywheel's range (1458) the hood wheel wants 2916, far under its cap. The FLYWHEEL is the
    //   wheel that runs out first now. Asking for more than 1458 flywheel RPM is cut down to 1458,
    //   and then the hood wheel (still twice what you asked for) is off ratio.
    //
    // 2.0 is the best ratio found so far in testing on the robot (no-spin shots). It is the
    // number TeleOp starts its ratio dial at. Keep testing a little higher or lower with
    // ShooterRatioAlpha, and type any better number here.
    private static final double HOOD_WHEEL_RATIO = 2.0;

    // A wheel counts as "at speed" when it is within this many RPM of its target.
    // A smaller number means a more precise shot but a longer wait. It is 25, the same size as
    // one step of the shot-table dial, because the shooter gate will only open when READY.
    // BOTH wheels must be inside it, and the hood wheel runs at about twice the flywheel's RPM.
    // WATCH FOR READY FLICKERING: one reading outside the band drops READY at once (on purpose),
    // so if the RPM on the screen wanders by more than about 15 RPM while the wheels are
    // steady, READY will keep blinking off. If that happens, the cause is the speed reading or
    // the speed control (PIDF), not the tolerance. Tell the coach before loosening this.
    private static final double AT_SPEED_TOLERANCE_RPM = 25.0;

    // ------------------------------------------------------------------
    // Smoothing the RPM reading
    // ------------------------------------------------------------------
    // The Control Hub works out each wheel's speed from the few encoder counts that arrive in a
    // very short time, so a single reading jumps around. A steady wheel at 1300 RPM can read
    // anywhere from about 1290 to 1330 from one loop to the next. The wheel itself is too heavy
    // to change speed that fast, so most of that jump is noise in the READING, not the wheel.
    // We smooth it: each loop, the new reading is blended into a running average. (This is
    // called an exponential low-pass filter.)
    //
    // RPM_FILTER_TIME_CONSTANT_SECONDS is how long the average takes to catch up with a real
    // change in speed. About 0.1 means it catches up most of the way in about a third of a
    // second.
    //   Bigger  -> a steadier number, but it is slower to show a real change.
    //   Smaller -> a jumpier number that reacts faster. 0.0 turns the smoothing off.
    // WATCH OUT: when a ball goes through, the flywheel really does slow down for a moment, and
    // READY drops when that happens. A bigger number here can smooth that dip away or delay it.
    // If READY stays on while a ball is going through, make this number smaller.
    // Starting guess. Tune on the robot.
    private static final double RPM_FILTER_TIME_CONSTANT_SECONDS = 0.10;

    // If more than this many seconds pass between update() calls, the old average is out of date,
    // so we start the average over from the new reading.
    private static final double RPM_FILTER_MAX_GAP_SECONDS = 0.5;

    // BOTH wheels must stay within the tolerance this long before we say READY.
    // This stops READY from flashing on while the wheels are just passing through the speed.
    // Starting guess.
    private static final double AT_SPEED_HOLD_SECONDS = 0.15;

    // ------------------------------------------------------------------
    // Speed-control (PIDF) settings
    // ------------------------------------------------------------------
    // Leave this false to start. The Control Hub then uses its own built-in numbers for
    // the motor type you picked in the configuration. If the wheel is slow to reach
    // speed, overshoots, or wobbles, set it to true and tune the four numbers below.
    // What the four numbers do:
    //   F = a first guess of the power needed for a speed (the biggest piece).
    //   P = pushes harder the further the wheel is from the target speed.
    //   I = slowly fixes a small error that will not go away.
    //   D = calms the push to stop overshooting.
    // Change ONE number at a time, and change it by a small amount.
    private static final boolean USE_CUSTOM_PIDF = false;

    // The Hub's full-power value is 32767. F = 32767 / (the motor's top speed in ticks/sec).
    // This is the usual starting point for F. It is worked out from the numbers above,
    // so if you change the ticks per rev, F changes with it. Each wheel has its own top speed
    // and its own ticks per rev, so each gets its own F. (They happen to come out almost the
    // same: both motors make about 2,800 ticks/sec at full power.)
    private static final double HUB_FULL_POWER_VALUE = 32767.0;
    private static final double FLYWHEEL_MAX_TICKS_PER_SECOND =
            FLYWHEEL_MOTOR_MAX_RPM / SECONDS_PER_MINUTE * FLYWHEEL_TICKS_PER_REV;
    private static final double HOOD_WHEEL_MAX_TICKS_PER_SECOND =
            HOOD_WHEEL_MOTOR_MAX_RPM / SECONDS_PER_MINUTE * HOOD_WHEEL_TICKS_PER_REV;
    private static final double VELOCITY_P = 10.0;   // Starting guess. Tune on the robot.
    private static final double VELOCITY_I = 0.0;    // Starting guess. Tune on the robot.
    private static final double VELOCITY_D = 0.0;    // Starting guess. Tune on the robot.
    private static final double FLYWHEEL_VELOCITY_F =
            HUB_FULL_POWER_VALUE / FLYWHEEL_MAX_TICKS_PER_SECOND;
    private static final double HOOD_WHEEL_VELOCITY_F =
            HUB_FULL_POWER_VALUE / HOOD_WHEEL_MAX_TICKS_PER_SECOND;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    // STATE = what we told the shooter to do (and whether it has got there yet).
    public enum ShooterState {
        IDLE,         // Stopped. Both wheels are coasting.
        OPEN_LOOP,    // Running at a raw power (setPower). No speed control.
        SPINNING_UP,  // Speed control is on, but the wheels are not at speed yet.
        READY         // Speed control is on and BOTH wheels have been at speed for a short time.
    }

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    // DcMotorEx is a DcMotor with extra features, like reading and setting speed (velocity).
    private DcMotorEx flywheelMotor;
    private DcMotorEx hoodWheelMotor;

    private ShooterState state = ShooterState.IDLE;

    // True while the motors are in speed-control mode (RUN_USING_ENCODER).
    // We remember this so we only change the motor mode when it really changes.
    private boolean speedControlOn = false;

    // What we asked for, in RPM. Both are 0 when stopped or in open loop.
    private double targetFlywheelRpm = 0.0;
    private double targetHoodWheelRpm = 0.0;

    // What the encoders reported the last time update() ran, in ticks per second.
    // This is the RAW reading, with no smoothing.
    private double flywheelTicksPerSecond = 0.0;
    private double hoodWheelTicksPerSecond = 0.0;

    // The SMOOTHED speed of each wheel, in RPM. update() works these out from the raw readings.
    // The READY check and the RPM on the screen use these, not the raw readings.
    private double flywheelRpm = 0.0;
    private double hoodWheelRpm = 0.0;
    private ElapsedTime rpmFilterTimer = new ElapsedTime();
    private boolean rpmFilterStarted = false;

    // Times how long both wheels have been within the tolerance.
    private ElapsedTime atSpeedTimer = new ElapsedTime();
    private boolean wasWithinTolerance = false;

    // The same check, but for the FLYWHEEL ONLY (see isFlywheelAtSpeed()). It uses the same
    // AT_SPEED_TOLERANCE_RPM and AT_SPEED_HOLD_SECONDS, so there is no second copy of the numbers.
    private ElapsedTime flywheelAtSpeedTimer = new ElapsedTime();
    private boolean flywheelWasWithinTolerance = false;
    private boolean flywheelAtSpeed = false;

    /*
     * Setup. Gets the motors from the robot configuration and sets them up.
     * The shooter starts stopped (IDLE).
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {
        // The names MUST match the robot configuration exactly.
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        hoodWheelMotor = hardwareMap.get(DcMotorEx.class, HOOD_WHEEL_MOTOR_NAME);

        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
        hoodWheelMotor.setDirection(HOOD_WHEEL_DIRECTION);

        // FLOAT lets the wheels coast down. Do NOT use BRAKE on a heavy flywheel:
        // stopping it suddenly puts a lot of stress on the motor and gears.
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hoodWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (USE_CUSTOM_PIDF) {
            flywheelMotor.setVelocityPIDFCoefficients(
                    VELOCITY_P, VELOCITY_I, VELOCITY_D, FLYWHEEL_VELOCITY_F);
            hoodWheelMotor.setVelocityPIDFCoefficients(
                    VELOCITY_P, VELOCITY_I, VELOCITY_D, HOOD_WHEEL_VELOCITY_F);
        }

        // Start in open loop at zero power. The encoder still reports speed in this mode.
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hoodWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setPower(0.0);
        hoodWheelMotor.setPower(0.0);
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the shooter what to do
    // ==================================================================

    /*
     * Spin the flywheel to a target RPM. The hood wheel follows at
     * HOOD_WHEEL_RATIO times that speed. Use this one for most shots.
     */
    public void setTargetRpm(double flywheelRpm) {
        setTargetRpms(flywheelRpm, flywheelRpm * HOOD_WHEEL_RATIO);
    }

    /*
     * Spin the flywheel and the hood wheel to their OWN target RPMs.
     *
     * Safe to call every loop with the same numbers. A repeat call changes nothing,
     * so a held button will not keep restarting the READY check.
     *
     * RPM here is never negative. To run backwards (to clear a jam), use setPower().
     * Asking for 0 on BOTH wheels is the same as stop(), so they coast down.
     * Asking for 0 on only ONE wheel makes the controller hold that wheel at zero
     * speed, which works like a brake. Use stop() instead if you want it to coast.
     */
    public void setTargetRpms(double flywheelRpm, double hoodWheelRpm) {
        double newFlywheelRpm = Range.clip(flywheelRpm, 0.0, FLYWHEEL_MAX_TARGET_RPM);
        double newHoodWheelRpm = Range.clip(hoodWheelRpm, 0.0, HOOD_WHEEL_MAX_TARGET_RPM);

        // Both zero means "stop". Never command a speed of 0 to a heavy flywheel,
        // because the controller would fight the coast-down like a brake.
        if (newFlywheelRpm == 0.0 && newHoodWheelRpm == 0.0) {
            stop();
            return;
        }

        // Same request as last time? Then there is nothing to do.
        if (speedControlOn
                && newFlywheelRpm == targetFlywheelRpm
                && newHoodWheelRpm == targetHoodWheelRpm) {
            return;
        }

        turnSpeedControlOn();
        targetFlywheelRpm = newFlywheelRpm;
        targetHoodWheelRpm = newHoodWheelRpm;
        flywheelMotor.setVelocity(rpmToTicksPerSecond(newFlywheelRpm, FLYWHEEL_TICKS_PER_REV));
        hoodWheelMotor.setVelocity(rpmToTicksPerSecond(newHoodWheelRpm, HOOD_WHEEL_TICKS_PER_REV));

        // New target, so start over: wait for the wheels to get there.
        wasWithinTolerance = false;
        flywheelWasWithinTolerance = false;
        flywheelAtSpeed = false;
        state = ShooterState.SPINNING_UP;
    }

    /*
     * Run the wheels at a raw power from -1.0 to 1.0 with NO speed control.
     * Shooting is POSITIVE. Negative runs the wheel backwards, for clearing a jam.
     * Zero power on BOTH wheels is the same as stop().
     */
    public void setPower(double flywheelPower, double hoodWheelPower) {
        if (flywheelPower == 0.0 && hoodWheelPower == 0.0) {
            stop();
            return;
        }

        turnSpeedControlOff();
        flywheelMotor.setPower(Range.clip(flywheelPower, -1.0, 1.0));
        hoodWheelMotor.setPower(Range.clip(hoodWheelPower, -1.0, 1.0));

        targetFlywheelRpm = 0.0;
        targetHoodWheelRpm = 0.0;
        wasWithinTolerance = false;
        flywheelWasWithinTolerance = false;
        flywheelAtSpeed = false;
        state = ShooterState.OPEN_LOOP;
    }

    /*
     * Stop both wheels and let them coast (FLOAT). Safe to call every loop.
     * We switch to power mode and set power 0. We do NOT set a speed of 0,
     * because that would make the controller brake the heavy flywheel.
     */
    public void stop() {
        turnSpeedControlOff();
        flywheelMotor.setPower(0.0);
        hoodWheelMotor.setPower(0.0);

        targetFlywheelRpm = 0.0;
        targetHoodWheelRpm = 0.0;
        wasWithinTolerance = false;
        flywheelWasWithinTolerance = false;
        flywheelAtSpeed = false;
        state = ShooterState.IDLE;
    }

    /*
     * Call this ONCE per loop(), after you give the shooter its commands.
     * It reads the encoders, and while speed control is on it decides
     * whether we are SPINNING_UP or READY.
     */
    public void update() {
        // Read the speeds once, so everything in this loop uses the same numbers.
        flywheelTicksPerSecond = flywheelMotor.getVelocity();
        hoodWheelTicksPerSecond = hoodWheelMotor.getVelocity();
        smoothRpms();

        // The flywheel-only check runs every loop, whatever the state (see isFlywheelAtSpeed()).
        updateFlywheelOnlyCheck();

        // Only speed control has a target to check against.
        if (state != ShooterState.SPINNING_UP && state != ShooterState.READY) {
            return;
        }

        double flywheelError = Math.abs(getFlywheelRpm() - targetFlywheelRpm);
        double hoodWheelError = Math.abs(getHoodWheelRpm() - targetHoodWheelRpm);
        boolean withinTolerance = flywheelError <= AT_SPEED_TOLERANCE_RPM
                && hoodWheelError <= AT_SPEED_TOLERANCE_RPM;

        if (!withinTolerance) {
            // Too slow or too fast. This also happens right after a shot, when the
            // ball slows the flywheel. We leave READY at once, so nobody feeds a ball early.
            wasWithinTolerance = false;
            state = ShooterState.SPINNING_UP;
            return;
        }

        // Within tolerance. Start the timer the first time, then wait out the hold time.
        if (!wasWithinTolerance) {
            atSpeedTimer.reset();
            wasWithinTolerance = true;
        }
        if (atSpeedTimer.seconds() >= AT_SPEED_HOLD_SECONDS) {
            state = ShooterState.READY;
        }
    }

    /*
     * The flywheel-only version of the at-speed check in update(). It looks at the flywheel's
     * smoothed RPM and its target, nothing else. Called once per loop from update().
     */
    private void updateFlywheelOnlyCheck() {
        // Only speed control has a target to check against.
        if (state != ShooterState.SPINNING_UP && state != ShooterState.READY) {
            flywheelWasWithinTolerance = false;
            flywheelAtSpeed = false;
            return;
        }

        double flywheelError = Math.abs(getFlywheelRpm() - targetFlywheelRpm);
        if (flywheelError > AT_SPEED_TOLERANCE_RPM) {
            // Too slow or too fast. This also happens right after a shot, when the ball slows the
            // flywheel.
            flywheelWasWithinTolerance = false;
            flywheelAtSpeed = false;
            return;
        }

        // Within tolerance. Start the timer the first time, then wait out the hold time.
        if (!flywheelWasWithinTolerance) {
            flywheelAtSpeedTimer.reset();
            flywheelWasWithinTolerance = true;
        }
        flywheelAtSpeed = flywheelAtSpeedTimer.seconds() >= AT_SPEED_HOLD_SECONDS;
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the shooter what is happening
    // ==================================================================

    /*
     * True only when speed control is on and BOTH wheels are at speed.
     * This is the shooter's part of the "OK to feed a ball" check.
     */
    public boolean isAtSpeed() {
        return state == ShooterState.READY;
    }

    /*
     * True when speed control is on and the FLYWHEEL ONLY has been within AT_SPEED_TOLERANCE_RPM of
     * its target for AT_SPEED_HOLD_SECONDS. The hood wheel is NOT checked.
     *
     * WHY THIS EXISTS: isAtSpeed() needs BOTH wheels, and the hood wheel's speed reading is a lot
     * noisier (28 encoder counts per turn, against the flywheel's 103.8, and it runs about twice as
     * fast). In the first Auto robot test isAtSpeed() only came true about 4 seconds after the
     * flywheel started. The firing sequence uses this one and shows the hood wheel's error, so the
     * shot is not held up by a noisy reading. isAtSpeed() is NOT changed, so it still checks both.
     * (The coach decides which one the TeleOp's "OK to shoot" uses. See the hood wheel issue.)
     *
     * Like isAtSpeed(), one reading outside the band drops it at once, and it is only fresh once
     * update() has run this loop.
     */
    public boolean isFlywheelAtSpeed() {
        return flywheelAtSpeed;
    }

    public ShooterState getState() {
        return state;
    }

    // Actual speed of each wheel in RPM, as of the last update(). These are SMOOTHED (see
    // RPM_FILTER_TIME_CONSTANT_SECONDS). The raw reading is getFlywheelTicksPerSecond().
    public double getFlywheelRpm() {
        return flywheelRpm;
    }

    public double getHoodWheelRpm() {
        return hoodWheelRpm;
    }

    // The raw encoder speed. Use this to check the ticks per rev (see the note near the top).
    public double getFlywheelTicksPerSecond() {
        return flywheelTicksPerSecond;
    }

    public double getHoodWheelTicksPerSecond() {
        return hoodWheelTicksPerSecond;
    }

    // What we asked for. Both are 0 when stopped or in open loop.
    public double getTargetFlywheelRpm() {
        return targetFlywheelRpm;
    }

    public double getTargetHoodWheelRpm() {
        return targetHoodWheelRpm;
    }

    // The highest RPM setTargetRpms() will let each wheel go to. Asking for more is quietly
    // cut down to this number. OpModes use these to warn that a target was cut down.
    public double getMaxFlywheelTargetRpm() {
        return FLYWHEEL_MAX_TARGET_RPM;
    }

    public double getMaxHoodWheelTargetRpm() {
        return HOOD_WHEEL_MAX_TARGET_RPM;
    }

    /*
     * Adds the shooter's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- SHOOTER ---");
        telemetry.addData("Shooter State", state);
        telemetry.addData("At Speed?", isAtSpeed());
        telemetry.addData("Flywheel RPM (smoothed)", "%.0f  (target %.0f)", getFlywheelRpm(), targetFlywheelRpm);
        telemetry.addData("Hood Wheel RPM (smoothed)", "%.0f  (target %.0f)", getHoodWheelRpm(), targetHoodWheelRpm);
        telemetry.addData("Flywheel ticks/sec (raw)", "%.0f", flywheelTicksPerSecond);
        telemetry.addData("Hood Wheel ticks/sec (raw)", "%.0f", hoodWheelTicksPerSecond);
    }

    // ==================================================================
    // PRIVATE HELPERS
    // ==================================================================

    // Speed control needs RUN_USING_ENCODER. In that mode the motor controller adjusts
    // power by itself to hold the speed we ask for with setVelocity().
    private void turnSpeedControlOn() {
        if (speedControlOn) {
            return;
        }
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        speedControlOn = true;
    }

    // Open loop needs RUN_WITHOUT_ENCODER. In RUN_USING_ENCODER, even setPower() turns into
    // speed control, so we must leave that mode before we use raw power.
    private void turnSpeedControlOff() {
        if (!speedControlOn) {
            return;
        }
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hoodWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        speedControlOn = false;
    }

    /*
     * Blends this loop's raw readings into the smoothed RPMs. Call it once per update(), right
     * after reading the encoders.
     */
    private void smoothRpms() {
        double rawFlywheelRpm = ticksPerSecondToRpm(flywheelTicksPerSecond, FLYWHEEL_TICKS_PER_REV);
        double rawHoodWheelRpm = ticksPerSecondToRpm(hoodWheelTicksPerSecond, HOOD_WHEEL_TICKS_PER_REV);

        double secondsSinceLastUpdate = rpmFilterTimer.seconds();
        rpmFilterTimer.reset();

        // blend is how much of the NEW reading goes into the average (0.0 to 1.0).
        // 1.0 means "use the new reading as it is".
        double blend;
        if (!rpmFilterStarted || secondsSinceLastUpdate > RPM_FILTER_MAX_GAP_SECONDS) {
            // The first reading, or a long gap: there is no good average yet, so start fresh.
            blend = 1.0;
            rpmFilterStarted = true;
        } else {
            // A longer time since the last loop means the new reading counts for more.
            // With a time constant of 0 this is 1.0, so there is no smoothing.
            blend = secondsSinceLastUpdate
                    / (RPM_FILTER_TIME_CONSTANT_SECONDS + secondsSinceLastUpdate);
        }

        flywheelRpm = flywheelRpm + blend * (rawFlywheelRpm - flywheelRpm);
        hoodWheelRpm = hoodWheelRpm + blend * (rawHoodWheelRpm - hoodWheelRpm);
    }

    // getVelocity() and setVelocity() work in encoder ticks per second.
    // RPM = (ticks per second / ticks per rev) x 60 seconds per minute.
    private double ticksPerSecondToRpm(double ticksPerSecond, double ticksPerRev) {
        return ticksPerSecond / ticksPerRev * SECONDS_PER_MINUTE;
    }

    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm / SECONDS_PER_MINUTE * ticksPerRev;
    }
}
