package org.firstinspires.ftc.teamcode.lib.motion;

import static org.firstinspires.ftc.teamcode.lib.motion.MotionUtil.kEpsilon;
import static org.firstinspires.ftc.teamcode.lib.util.Util.epsilonEquals;

/**
 * A MotionState is a completely specified state of 1D motion through time.
 */
public class MotionState {
    protected final double t;
    protected final double pos;
    protected final double vel;
    protected final double acc;

    public static MotionState kInvalidState = new MotionState(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    public MotionState(double t, double pos, double vel, double acc) {
        this.t = t;
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
    }

    public MotionState(MotionState state) {
        this(state.t, state.pos, state.vel, state.acc);
    }

    public double t() {
        return t;
    }

    public double pos() {
        return pos;
    }

    public double vel() {
        return vel;
    }

    public double vel2() {
        return vel * vel;
    }

    public double acc() {
        return acc;
    }

    /**
     * Extrapolates this MotionState to the specified time by applying this MotionState's acceleration.
     *
     * @param t The time of the new MotionState.
     * @return A MotionState that is a valid predecessor (if t<=0) or successor (if t>=0) of this state.
     */
    public MotionState extrapolate(double t) {
        return extrapolate(t, acc);
    }

    /**
     * Extrapolates this MotionState to the specified time by applying a given acceleration to the (t, pos, vel) portion
     * of this MotionState.
     *
     * @param t   The time of the new MotionState.
     * @param acc The acceleration to apply.
     * @return A MotionState that is a valid predecessor (if t<=0) or successor (if t>=0) of this state (with the
     * specified accel).
     */
    public MotionState extrapolate(double t, double acc) {
        final double dt = t - this.t;
        return new MotionState(t, pos + vel * dt + .5 * acc * dt * dt, vel + acc * dt, acc);
    }

    /**
     * Find the next time (first time > MotionState.t()) that this MotionState will be at pos. This is an inverse of the
     * extrapolate() method.
     *
     * @param pos The position to query.
     * @return The time when we are next at pos() if we are extrapolating with a positive dt. NaN if we never reach pos.
     */
    public double nextTimeAtPos(double pos) {
        if (epsilonEquals(pos, this.pos, kEpsilon)) {
            // Already at pos.
            return t;
        }
        if (epsilonEquals(acc, 0.0, kEpsilon)) {
            // Zero acceleration case.
            final double delta_pos = pos - this.pos;
            if (!epsilonEquals(vel, 0.0, kEpsilon) && Math.signum(delta_pos) == Math.signum(vel)) {
                // Constant velocity heading towards pos.
                return delta_pos / vel + t;
            }
            return Double.NaN;
        }

        // Solve the quadratic formula.
        // ax^2 + bx + c == 0
        // x = dt
        // a = .5 * acc
        // b = vel
        // c = this.pos - pos
        final double disc = vel * vel - 2.0 * acc * (this.pos - pos);
        if (disc < 0.0) {
            // Extrapolating this MotionState never reaches the desired pos.
            return Double.NaN;
        }
        final double sqrt_disc = Math.sqrt(disc);
        final double max_dt = (-vel + sqrt_disc) / acc;
        final double min_dt = (-vel - sqrt_disc) / acc;
        if (min_dt >= 0.0 && (max_dt < 0.0 || min_dt < max_dt)) {
            return t + min_dt;
        }
        if (max_dt >= 0.0) {
            return t + max_dt;
        }
        // We only reach the desired pos in the past.
        return Double.NaN;
    }

    @Override
    public String toString() {
        return "(t=" + t + ", pos=" + pos + ", vel=" + vel + ", acc=" + acc + ")";
    }

    /**
     * Checks if two MotionStates are epsilon-equals (all fields are equal within a nominal tolerance).
     */
    @Override
    public boolean equals(Object other) {
        return (other instanceof MotionState) && equals((MotionState) other, kEpsilon);
    }

    /**
     * Checks if two MotionStates are epsilon-equals (all fields are equal within a specified tolerance).
     */
    public boolean equals(MotionState other, double epsilon) {
        return coincident(other, epsilon) && epsilonEquals(acc, other.acc, epsilon);
    }

    /**
     * Checks if two MotionStates are coincident (t, pos, and vel are equal within a nominal tolerance, but acceleration
     * may be different).
     */
    public boolean coincident(MotionState other) {
        return coincident(other, kEpsilon);
    }

    /**
     * Checks if two MotionStates are coincident (t, pos, and vel are equal within a specified tolerance, but
     * acceleration may be different).
     */
    public boolean coincident(MotionState other, double epsilon) {
        return epsilonEquals(t, other.t, epsilon) && epsilonEquals(pos, other.pos, epsilon)
                && epsilonEquals(vel, other.vel, epsilon);
    }

    /**
     * Returns a MotionState that is the mirror image of this one. Pos, vel, and acc are all negated, but time is not.
     */
    public MotionState flipped() {
        return new MotionState(t, -pos, -vel, -acc);
    }
}
