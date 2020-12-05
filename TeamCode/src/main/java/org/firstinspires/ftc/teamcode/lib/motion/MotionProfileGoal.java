package org.firstinspires.ftc.teamcode.lib.motion;

import static org.firstinspires.ftc.teamcode.lib.util.Util.epsilonEquals;

/**
 * A MotionProfileGoal defines a desired position and maximum velocity (at this position), along with the behavior that
 * should be used to determine if we are at the goal and what to do if it is infeasible to reach the goal within the
 * desired velocity bounds.
 */
public class MotionProfileGoal {
    /**
     * A goal consists of a desired position and specified maximum velocity magnitude. But what should we do if we would
     * reach the goal at a velocity greater than the maximum? This enum allows a user to specify a preference on
     * behavior in this case.
     * <p>
     * Example use-cases of each:
     * <p>
     * OVERSHOOT - Generally used with a goal max_abs_vel of 0.0 to stop at the desired pos without violating any
     * constraints.
     * <p>
     * VIOLATE_MAX_ACCEL - If we absolutely do not want to pass the goal and are unwilling to violate the max_abs_vel
     * (for example, there is an obstacle in front of us - slam the brakes harder than we'd like in order to avoid
     * hitting it).
     * <p>
     * VIOLATE_MAX_ABS_VEL - If the max velocity is just a general guideline and not a hard performance limit, it's
     * better to slightly exceed it to avoid skidding wheels.
     */
    public enum CompletionBehavior {
        // Overshoot the goal if necessary (at a velocity greater than max_abs_vel) and come back.
        // Only valid if the goal velocity is 0.0 (otherwise VIOLATE_MAX_ACCEL will be used).
        OVERSHOOT,
        // If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the max accel
        // constraint.
        VIOLATE_MAX_ACCEL,
        // If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the goal velocity.
        VIOLATE_MAX_ABS_VEL
    }

    protected double pos;
    protected double max_abs_vel;
    protected CompletionBehavior completion_behavior = CompletionBehavior.OVERSHOOT;
    protected double pos_tolerance = 1E-3;
    protected double vel_tolerance = 1E-2;

    public MotionProfileGoal() {}

    public MotionProfileGoal(double pos) {
        this.pos = pos;
        this.max_abs_vel = 0.0;
        sanityCheck();
    }

    public MotionProfileGoal(double pos, double max_abs_vel) {
        this.pos = pos;
        this.max_abs_vel = max_abs_vel;
        sanityCheck();
    }

    public MotionProfileGoal(double pos, double max_abs_vel, CompletionBehavior completion_behavior) {
        this.pos = pos;
        this.max_abs_vel = max_abs_vel;
        this.completion_behavior = completion_behavior;
        sanityCheck();
    }

    public MotionProfileGoal(double pos, double max_abs_vel, CompletionBehavior completion_behavior,
                             double pos_tolerance, double vel_tolerance) {
        this.pos = pos;
        this.max_abs_vel = max_abs_vel;
        this.completion_behavior = completion_behavior;
        this.pos_tolerance = pos_tolerance;
        this.vel_tolerance = vel_tolerance;
        sanityCheck();
    }

    public MotionProfileGoal(MotionProfileGoal other) {
        this(other.pos, other.max_abs_vel, other.completion_behavior, other.pos_tolerance, other.vel_tolerance);
    }

    /**
     * @return A flipped MotionProfileGoal (where the position is negated, but all other attributes remain the same).
     */
    public MotionProfileGoal flipped() {
        return new MotionProfileGoal(-pos, max_abs_vel, completion_behavior, pos_tolerance, vel_tolerance);
    }

    public double pos() {
        return pos;
    }

    public double max_abs_vel() {
        return max_abs_vel;
    }

    public double pos_tolerance() {
        return pos_tolerance;
    }

    public double vel_tolerance() {
        return vel_tolerance;
    }

    public CompletionBehavior completion_behavior() {
        return completion_behavior;
    }

    public boolean atGoalState(MotionState state) {
        return atGoalPos(state.pos()) && (Math.abs(state.vel()) < (max_abs_vel + vel_tolerance)
                || completion_behavior == CompletionBehavior.VIOLATE_MAX_ABS_VEL);
    }

    public boolean atGoalPos(double pos) {
        return epsilonEquals(pos, this.pos, pos_tolerance);
    }

    /**
     * This method makes sure that the completion behavior is compatible with the max goal velocity.
     */
    protected void sanityCheck() {
        if (max_abs_vel > vel_tolerance && completion_behavior == CompletionBehavior.OVERSHOOT) {
            completion_behavior = CompletionBehavior.VIOLATE_MAX_ACCEL;
        }
    }

    @Override
    public String toString() {
        return "pos: " + pos + " (+/- " + pos_tolerance + "), max_abs_vel: " + max_abs_vel + " (+/- " + vel_tolerance
                + "), completion behavior: " + completion_behavior.name();
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MotionProfileGoal)) {
            return false;
        }
        final MotionProfileGoal other = (MotionProfileGoal) obj;
        return (other.completion_behavior() == completion_behavior()) && (other.pos() == pos())
                && (other.max_abs_vel() == max_abs_vel()) && (other.pos_tolerance() == pos_tolerance())
                && (other.vel_tolerance() == vel_tolerance());
    }
}
