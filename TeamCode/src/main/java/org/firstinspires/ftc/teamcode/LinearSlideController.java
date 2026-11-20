package org.firstinspires.ftc.teamcode;

/**
 * Hardware-free slide control: counts Axon analog turns and drives toward fully down (0) or
 * fully up ({@link #MAX_TRAVEL_DEG}). Positive power and positive position both mean UP.
 * Called once per loop; never blocks.
 */
public final class LinearSlideController {

    public enum State { DOWN, MOVING_UP, UP, MOVING_DOWN, MANUAL, FAULT }

    // Servo-output degrees from fully down (INIT) to fully up. PLACEHOLDER: deliberately short
    // (half a turn) so the first power-up cannot over-travel. Measure with "Test: Linear Slide".
    public static final double MAX_TRAVEL_DEG = 180.0;
    public static final boolean TRAVEL_MEASURED = false;

    // Proportional power per degree of error; 100 deg of error requests full power.
    public static final double KP_POWER_PER_DEG = 0.01;
    public static final double MAX_POWER = 1.0;
    // Smallest power used while moving, so the CR servo does not stall in its neutral deadband.
    public static final double MIN_MOVING_POWER = 0.10;
    public static final double TOLERANCE_DEG = 3.0;
    // Once settled, only correct again after drifting this far.
    public static final double HOLD_RECOVER_DEG = 2 * TOLERANCE_DEG;
    // Extra power while holding UP, if the slide sags under gravity. 0 = unpowered hold.
    public static final double HOLD_POWER_UP = 0.0;
    // Never drive further past either end than this.
    public static final double LIMIT_MARGIN_DEG = 10.0;

    // Faults. A stall is less than STALL_MIN_PROGRESS_DEG of travel within STALL_WINDOW_SECONDS.
    public static final double STALL_WINDOW_SECONDS = 0.5;
    public static final double STALL_MIN_PROGRESS_DEG = 5.0;
    // Moving this far the wrong way means SERVO_DIRECTION / ENCODER_INVERTED disagree.
    public static final double WRONG_WAY_DEG = 30.0;
    // Overall limit: 1 s plus the travel at a quarter of the rated Axon MAX+ speed.
    public static final double MOVE_TIMEOUT_SECONDS =
            1.0 + MAX_TRAVEL_DEG / (0.25 * AxonMaxPlus.DEGREES_PER_SECOND_AT_6V);
    // The analog angle is ambiguous past half a turn per loop.
    public static final double MAX_UNWRAP_STEP_DEG = 180.0;

    private final boolean encoderInverted;
    private final double maxTravelDeg;
    private final double moveTimeoutSeconds;
    private boolean hasReading = false;
    private double previousRawDeg;
    private double positionDeg;
    private boolean upCommanded = false;
    private State state = State.DOWN;
    private double power = 0.0;
    private double manualPower = Double.NaN;
    private boolean moving = false;
    private double moveStartedAt, moveStartError;
    private double progressCheckAt, progressCheckPositionDeg;
    private String faultReason = "";

    public LinearSlideController(boolean encoderInverted) {
        this(encoderInverted, MAX_TRAVEL_DEG, MOVE_TIMEOUT_SECONDS);
    }

    LinearSlideController(boolean encoderInverted, double maxTravelDeg, double moveTimeoutSeconds) {
        if (!(maxTravelDeg > 0) || !(moveTimeoutSeconds > 0))
            throw new IllegalArgumentException("Invalid slide travel or timeout");
        this.encoderInverted = encoderInverted;
        this.maxTravelDeg = maxTravelDeg;
        this.moveTimeoutSeconds = moveTimeoutSeconds;
    }

    /** Declares the current analog angle as fully down (position 0). */
    public void zero(double rawDeg) {
        previousRawDeg = rawDeg;
        positionDeg = 0.0;
        hasReading = true;
    }

    public void toggle(double now) { command(!upCommanded, now); }
    public void raise(double now) { command(true, now); }
    public void lower(double now) { command(false, now); }

    private void command(boolean up, double now) {
        upCommanded = up;
        manualPower = Double.NaN;
        faultReason = "";
        moving = true;
        moveStartedAt = progressCheckAt = now;
        progressCheckPositionDeg = positionDeg;
        moveStartError = Math.abs(targetDeg() - positionDeg);
        state = up ? State.MOVING_UP : State.MOVING_DOWN;
    }

    /**
     * Test only: open-loop power with no limits, for finding the top. NaN releases the stick:
     * the slide stays unpowered where it is (no drive back to the old target) until the next command.
     */
    public void setManualPower(double power, double now) {
        if (Double.isNaN(power)) {
            if (!Double.isNaN(manualPower)) manualPower = 0.0;
            return;
        }
        manualPower = Math.max(-1.0, Math.min(1.0, power));
        moving = false;
        faultReason = "";
        state = State.MANUAL;
    }

    /** Stops output until the next command. */
    public void stop() {
        moving = false;
        manualPower = Double.NaN;
        power = 0.0;
        if (state != State.FAULT) state = upCommanded ? State.UP : State.DOWN;
    }

    /** Unwraps one analog reading (counts turns) without computing power. */
    public void observe(double rawDeg) {
        if (!hasReading) zero(rawDeg);
        double delta = rawDeg - previousRawDeg;
        while (delta > MAX_UNWRAP_STEP_DEG) delta -= 360.0;
        while (delta < -MAX_UNWRAP_STEP_DEG) delta += 360.0;
        previousRawDeg = rawDeg;
        positionDeg += encoderInverted ? -delta : delta;
    }

    /** Unwraps one analog reading and returns this loop's CR servo power. */
    public double update(double now, double rawDeg) {
        observe(rawDeg);
        if (!Double.isNaN(manualPower)) return power = manualPower;
        if (state == State.FAULT) return power = 0.0;

        double error = targetDeg() - positionDeg;
        if (moving) {
            if (Math.abs(error) <= TOLERANCE_DEG) {
                moving = false;
            } else if (Math.abs(error) > moveStartError + WRONG_WAY_DEG) {
                return fault("moving wrong way: check SERVO_DIRECTION / ENCODER_INVERTED");
            } else if (now - moveStartedAt > moveTimeoutSeconds) {
                return fault(String.format(java.util.Locale.US, "not there after %.1f s", moveTimeoutSeconds));
            } else if (now - progressCheckAt >= STALL_WINDOW_SECONDS) {
                if (Math.abs(positionDeg - progressCheckPositionDeg) < STALL_MIN_PROGRESS_DEG)
                    return fault("stalled: jam, unplugged servo or unplugged encoder");
                progressCheckAt = now;
                progressCheckPositionDeg = positionDeg;
            }
        }

        if (moving) {
            state = upCommanded ? State.MOVING_UP : State.MOVING_DOWN;
            power = proportional(error);
        } else {
            state = upCommanded ? State.UP : State.DOWN;
            // Settled: hold, and only nudge back after a noticeable drift (no timeout here).
            power = Math.abs(error) > HOLD_RECOVER_DEG ? proportional(error)
                    : upCommanded ? HOLD_POWER_UP : 0.0;
        }
        if (positionDeg > maxTravelDeg + LIMIT_MARGIN_DEG && power > 0) power = 0.0;
        if (positionDeg < -LIMIT_MARGIN_DEG && power < 0) power = 0.0;
        return power;
    }

    private double proportional(double error) {
        double magnitude = Math.max(MIN_MOVING_POWER, Math.min(MAX_POWER, Math.abs(KP_POWER_PER_DEG * error)));
        return Math.copySign(magnitude, error);
    }

    private double fault(String reason) {
        moving = false;
        state = State.FAULT;
        faultReason = reason;
        return power = 0.0;
    }

    public double targetDeg() { return upCommanded ? maxTravelDeg : 0.0; }
    public double getPositionDeg() { return positionDeg; }
    public double getPower() { return power; }
    public State getState() { return state; }
    public String getFaultReason() { return faultReason; }
    public boolean isUpCommanded() { return upCommanded; }
    public double getMaxTravelDeg() { return maxTravelDeg; }
}
