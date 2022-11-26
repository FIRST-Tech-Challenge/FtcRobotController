package com.acmerobotics.roadrunner.profile

/**
 * Segment of a motion profile with constant acceleration.
 *
 * @param start start motion state
 * @param dt time delta
 */
class MotionSegment(val start: MotionState, val dt: Double) {

    /**
     * Returns the [MotionState] at time [t].
     */
    operator fun get(t: Double) = start[t]

    /**
     * Returns the [MotionState] at the end of the segment (time [dt]).
     */
    fun end() = start[dt]

    /**
     * Returns a reversed version of the segment. Note: it isn't possible to reverse a segment completely so this
     * method only guarantees that the start and end velocities will be swapped.
     */
    fun reversed(): MotionSegment {
        val end = end()
        val state = MotionState(end.x, end.v, -end.a, end.j)
        return MotionSegment(state, dt)
    }

    /**
     * Returns a flipped (negated) version of the segment.
     */
    fun flipped() = MotionSegment(start.flipped(), dt)

    override fun toString() = "($start, $dt)"
}
