package com.acmerobotics.roadrunner.profile

/**
 * Trapezoidal motion profile composed of motion segments.
 *
 * @param segments profile motion segments
 */
class MotionProfile(val segments: List<MotionSegment>) {
    init {
        assert(segments.isNotEmpty())
    }

    /**
     * Returns the [MotionState] at time [t].
     */
    operator fun get(t: Double): MotionState {
        if (t < 0.0) return segments.first().start.stationary()

        var remainingTime = t
        for (segment in segments) {
            if (remainingTime <= segment.dt) {
                return segment[remainingTime]
            }
            remainingTime -= segment.dt
        }

        return segments.last().end().stationary()
    }

    /**
     * Returns the duration of the motion profile.
     */
    fun duration() = segments.sumByDouble { it.dt }

    /**
     * Returns a reversed version of the motion profile.
     */
    fun reversed() = MotionProfile(segments.map { it.reversed() }.reversed())

    /**
     * Returns a flipped version of the motion profile.
     */
    fun flipped() = MotionProfile(segments.map { it.flipped() })

    /**
     * Returns the start [MotionState].
     */
    fun start() = segments.first().start

    /**
     * Returns the end [MotionState].
     */
    fun end() = segments.last().end()

    /**
     * Returns a new motion profile with [other] concatenated.
     */
    operator fun plus(other: MotionProfile): MotionProfile {
        val builder = MotionProfileBuilder(start())
        builder.appendProfile(this)
        builder.appendProfile(other)
        return builder.build()
    }
}
