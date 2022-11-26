package com.acmerobotics.roadrunner.profile

import com.acmerobotics.roadrunner.util.epsilonEquals

/**
 * Easy-to-use builder for creating motion profiles.
 *
 * @param start start motion state
 */
class MotionProfileBuilder(start: MotionState) {
    private var currentState = start
    private val segments = mutableListOf<MotionSegment>()

    /**
     * Appends a constant-jerk control for the provided duration.
     */
    fun appendJerkControl(jerk: Double, dt: Double): MotionProfileBuilder {
        val segment = MotionSegment(MotionState(currentState.x, currentState.v, currentState.a, jerk), dt)
        segments.add(segment)
        currentState = segment.end()
        return this
    }

    /**
     * Appends a constant-acceleration control for the provided duration.
     */
    fun appendAccelerationControl(accel: Double, dt: Double): MotionProfileBuilder {
        val segment = MotionSegment(MotionState(currentState.x, currentState.v, accel), dt)
        segments.add(segment)
        currentState = segment.end()
        return this
    }

    /**
     * Appends a [MotionProfile] to the current queue of control actions.
     */
    fun appendProfile(profile: MotionProfile): MotionProfileBuilder {
        for (segment in profile.segments) {
            if (segment.start.j epsilonEquals 0.0) {
                // constant acceleration
                appendAccelerationControl(segment.start.a, segment.dt)
            } else {
                // constant jerk
                appendJerkControl(segment.start.j, segment.dt)
            }
        }
        return this
    }

    /**
     * Constructs and returns the [MotionProfile] instance.
     */
    fun build() = MotionProfile(segments)
}
