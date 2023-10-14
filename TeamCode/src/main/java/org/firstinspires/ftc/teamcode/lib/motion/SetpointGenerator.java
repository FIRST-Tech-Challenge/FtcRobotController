package org.firstinspires.ftc.teamcode.lib.motion;

import java.util.Optional;

/**
 * A SetpointGenerate does just-in-time motion profile generation to supply a stream of setpoints that obey the given
 * constraints to a controller. The profile is regenerated when any of the inputs change, but is cached (and trimmed as
 * we go) if the only update is to the current state.
 * <p>
 * Note that typically for smooth control, a user will feed the last iteration's setpoint as the argument to
 * getSetpoint(), and should only use a measured state directly on the first iteration or if a large disturbance is
 * detected.
 */
public class SetpointGenerator {
    /**
     * A Setpoint is just a MotionState and an additional flag indicating whether this is setpoint achieves the goal
     * (useful for higher-level logic to know that it is now time to do something else).
     */
    public static class Setpoint {
        public MotionState motion_state;
        public boolean final_setpoint;

        public Setpoint(MotionState motion_state, boolean final_setpoint) {
            this.motion_state = motion_state;
            this.final_setpoint = final_setpoint;
        }
    }

    protected MotionProfile mProfile = null;
    protected MotionProfileGoal mGoal = null;
    protected MotionProfileConstraints mConstraints = null;

    public SetpointGenerator() {}

    /**
     * Force a reset of the profile.
     */
    public void reset() {
        mProfile = null;
        mGoal = null;
        mConstraints = null;
    }

    /**
     * Get a new Setpoint (and generate a new MotionProfile if necessary).
     *
     * @param constraints The constraints to use.
     * @param goal        The goal to use.
     * @param prev_state  The previous setpoint (or measured state of the system to do a reset).
     * @param t           The time to generate a setpoint for.
     * @return The new Setpoint at time t.
     */
    public synchronized Setpoint getSetpoint(MotionProfileConstraints constraints, MotionProfileGoal goal,
                                             MotionState prev_state,
                                             double t) {
        boolean regenerate = mConstraints == null || !mConstraints.equals(constraints) || mGoal == null
                || !mGoal.equals(goal) || mProfile == null;
        if (!regenerate && !mProfile.isEmpty()) {
            Optional<MotionState> expected_state = mProfile.stateByTime(prev_state.t());
            regenerate = !expected_state.isPresent() || !expected_state.get().equals(prev_state);
        }
        if (regenerate) {
            // Regenerate the profile, as our current profile does not satisfy the inputs.
            mConstraints = constraints;
            mGoal = goal;
            mProfile = MotionProfileGenerator.generateProfile(constraints, goal, prev_state);
            // System.out.println("Regenerating profile: " + mProfile);
        }

        // Sample the profile at time t.
        Setpoint rv = null;
        if (!mProfile.isEmpty() && mProfile.isValid()) {
            MotionState setpoint;
            if (t > mProfile.endTime()) {
                setpoint = mProfile.endState();
            } else if (t < mProfile.startTime()) {
                setpoint = mProfile.startState();
            } else {
                setpoint = mProfile.stateByTime(t).get();
            }
            // Shorten the profile and return the new setpoint.
            mProfile.trimBeforeTime(t);
            rv = new Setpoint(setpoint, mProfile.isEmpty() || mGoal.atGoalState(setpoint));
        }

        // Invalid or empty profile - just output the same state again.
        if (rv == null) {
            rv = new Setpoint(prev_state, true);
        }

        if (rv.final_setpoint) {
            // Ensure the final setpoint matches the goal exactly.
            rv.motion_state = new MotionState(rv.motion_state.t(), mGoal.pos(),
                    Math.signum(rv.motion_state.vel()) * Math.max(mGoal.max_abs_vel(), Math.abs(rv.motion_state.vel())),
                    0.0);
        }

        return rv;
    }

    /**
     * Get the full profile from the latest call to getSetpoint(). Useful to check estimated time or distance to goal.
     *
     * @return The profile from the latest call to getSetpoint(), or null if there is not yet a profile.
     */
    public MotionProfile getProfile() {
        return mProfile;
    }
}
