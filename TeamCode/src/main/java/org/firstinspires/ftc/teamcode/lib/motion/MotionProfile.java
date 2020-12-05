package org.firstinspires.ftc.teamcode.lib.motion;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import static org.firstinspires.ftc.teamcode.lib.motion.MotionUtil.kEpsilon;
import static org.firstinspires.ftc.teamcode.lib.util.Util.epsilonEquals;

/**
 * A motion profile specifies a 1D time-parameterized trajectory. The trajectory is composed of successively coincident
 * MotionSegments from which the desired state of motion at any given distance or time can be calculated.
 */
public class MotionProfile {
    protected List<MotionSegment> mSegments;

    /**
     * Create an empty MotionProfile.
     */
    public MotionProfile() {
        mSegments = new ArrayList<>();
    }

    /**
     * Create a MotionProfile from an existing list of segments (note that validity is not checked).
     *
     * @param segments The new segments of the profile.
     */
    public MotionProfile(List<MotionSegment> segments) {
        mSegments = segments;
    }

    /**
     * Checks if the given MotionProfile is valid. This checks that:
     * <p>
     * 1. All segments are valid.
     * <p>
     * 2. Successive segments are C1 continuous in position and C0 continuous in velocity.
     *
     * @return True if the MotionProfile is valid.
     */
    public boolean isValid() {
        MotionSegment prev_segment = null;
        for (MotionSegment s : mSegments) {
            if (!s.isValid()) {
                return false;
            }
            if (prev_segment != null && !s.start().coincident(prev_segment.end())) {
                // Adjacent segments are not continuous.
                System.err.println("Segments not continuous! End: " + prev_segment.end() + ", Start: " + s.start());
                return false;
            }
            prev_segment = s;
        }
        return true;
    }

    /**
     * Check if the profile is empty.
     *
     * @return True if there are no segments.
     */
    public boolean isEmpty() {
        return mSegments.isEmpty();
    }

    /**
     * Get the interpolated MotionState at any given time.
     *
     * @param t The time to query.
     * @return Empty if the time is outside the time bounds of the profile, or the resulting MotionState otherwise.
     */
    public Optional<MotionState> stateByTime(double t) {
        if (t < startTime() && t + kEpsilon >= startTime()) {
            return Optional.of(startState());
        }
        if (t > endTime() && t - kEpsilon <= endTime()) {
            return Optional.of(endState());
        }
        for (MotionSegment s : mSegments) {
            if (s.containsTime(t)) {
                return Optional.of(s.start().extrapolate(t));
            }
        }
        return Optional.empty();
    }

    /**
     * Get the interpolated MotionState at any given time, clamping to the endpoints if time is out of bounds.
     *
     * @param t The time to query.
     * @return The MotionState at time t, or closest to it if t is outside the profile.
     */
    public MotionState stateByTimeClamped(double t) {
        if (t < startTime()) {
            return startState();
        } else if (t > endTime()) {
            return endState();
        }
        for (MotionSegment s : mSegments) {
            if (s.containsTime(t)) {
                return s.start().extrapolate(t);
            }
        }
        // Should never get here.
        return MotionState.kInvalidState;
    }

    /**
     * Get the interpolated MotionState by distance (the "pos()" field of MotionState). Note that since a profile may
     * reverse, this method only returns the *first* instance of this position.
     *
     * @param pos The position to query.
     * @return Empty if the profile never crosses pos or if the profile is invalid, or the resulting MotionState
     * otherwise.
     */
    public Optional<MotionState> firstStateByPos(double pos) {
        for (MotionSegment s : mSegments) {
            if (s.containsPos(pos)) {
                if (epsilonEquals(s.end().pos(), pos, kEpsilon)) {
                    return Optional.of(s.end());
                }
                final double t = Math.min(s.start().nextTimeAtPos(pos), s.end().t());
                if (Double.isNaN(t)) {
                    System.err.println("Error! We should reach 'pos' but we don't");
                    return Optional.empty();
                }
                return Optional.of(s.start().extrapolate(t));
            }
        }
        // We never reach pos.
        return Optional.empty();
    }

    /**
     * Remove all parts of the profile prior to the query time. This eliminates whole segments and also shortens any
     * segments containing t.
     *
     * @param t The query time.
     */
    public void trimBeforeTime(double t) {
        for (Iterator<MotionSegment> iterator = mSegments.iterator(); iterator.hasNext(); ) {
            MotionSegment s = iterator.next();
            if (s.end().t() <= t) {
                // Segment is fully before t.
                iterator.remove();
                continue;
            }
            if (s.start().t() <= t) {
                // Segment begins before t; let's shorten the segment.
                s.setStart(s.start().extrapolate(t));
            }
            break;
        }
    }

    /**
     * Remove all segments.
     */
    public void clear() {
        mSegments.clear();
    }

    /**
     * Remove all segments and initialize to the desired state (actually a segment of length 0 that starts and ends at
     * initial_state).
     *
     * @param initial_state The MotionState to initialize to.
     */
    public void reset(MotionState initial_state) {
        clear();
        mSegments.add(new MotionSegment(initial_state, initial_state));
    }

    /**
     * Remove redundant segments (segments whose start and end states are coincident).
     */
    public void consolidate() {
        for (Iterator<MotionSegment> iterator = mSegments.iterator(); iterator.hasNext() && mSegments.size() > 1; ) {
            MotionSegment s = iterator.next();
            if (s.start().coincident(s.end())) {
                iterator.remove();
            }
        }
    }

    /**
     * Add to the profile by applying an acceleration control for a given time. This is appended to the previous last
     * state.
     *
     * @param acc The acceleration to apply.
     * @param dt  The period of time to apply the given acceleration.
     */
    public void appendControl(double acc, double dt) {
        if (isEmpty()) {
            System.err.println("Error!  Trying to append to empty profile");
            return;
        }
        MotionState last_end_state = mSegments.get(mSegments.size() - 1).end();
        MotionState new_start_state = new MotionState(last_end_state.t(), last_end_state.pos(), last_end_state.vel(),
                acc);
        appendSegment(new MotionSegment(new_start_state, new_start_state.extrapolate(new_start_state.t() + dt)));
    }

    /**
     * Add to the profile by inserting a new segment. No validity checking is done.
     *
     * @param segment The segment to add.
     */
    public void appendSegment(MotionSegment segment) {
        mSegments.add(segment);
    }

    /**
     * Add to the profile by inserting a new profile after the final state. No validity checking is done.
     *
     * @param profile The profile to add.
     */
    public void appendProfile(MotionProfile profile) {
        for (MotionSegment s : profile.segments()) {
            appendSegment(s);
        }
    }

    /**
     * @return The number of segments.
     */
    public int size() {
        return mSegments.size();
    }

    /**
     * @return The list of segments.
     */
    public List<MotionSegment> segments() {
        return mSegments;
    }

    /**
     * @return The first state in the profile (or kInvalidState if empty).
     */
    public MotionState startState() {
        if (isEmpty()) {
            return MotionState.kInvalidState;
        }
        return mSegments.get(0).start();
    }

    /**
     * @return The time of the first state in the profile (or NaN if empty).
     */
    public double startTime() {
        return startState().t();
    }

    /**
     * @return The pos of the first state in the profile (or NaN if empty).
     */
    public double startPos() {
        return startState().pos();
    }

    /**
     * @return The last state in the profile (or kInvalidState if empty).
     */
    public MotionState endState() {
        if (isEmpty()) {
            return MotionState.kInvalidState;
        }
        return mSegments.get(mSegments.size() - 1).end();
    }

    /**
     * @return The time of the last state in the profile (or NaN if empty).
     */
    public double endTime() {
        return endState().t();
    }

    /**
     * @return The pos of the last state in the profile (or NaN if empty).
     */
    public double endPos() {
        return endState().pos();
    }

    /**
     * @return The duration of the entire profile.
     */
    public double duration() {
        return endTime() - startTime();
    }

    /**
     * @return The total distance covered by the profile. Note that distance is the sum of absolute distances of all
     * segments, so a reversing profile will count the distance covered in each direction.
     */
    public double length() {
        double length = 0.0;
        for (MotionSegment s : segments()) {
            length += Math.abs(s.end().pos() - s.start().pos());
        }
        return length;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder("Profile:");
        for (MotionSegment s : segments()) {
            builder.append("\n\t");
            builder.append(s);
        }
        return builder.toString();
    }
}