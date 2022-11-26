package com.acmerobotics.roadrunner.profile

/**
 * Kinematic state of a motion profile at any given time.
 */
class MotionState @JvmOverloads constructor(
    val x: Double,
    val v: Double,
    val a: Double = 0.0,
    val j: Double = 0.0
) {

    /**
     * Returns the [MotionState] at time [t].
     */
    operator fun get(t: Double) =
        MotionState(
            x + v * t + a / 2 * t * t + j / 6 * t * t * t,
            v + a * t + j / 2 * t * t,
            a + j * t,
            j
        )

    /**
     * Returns a flipped (negated) version of the state.
     */
    fun flipped() = MotionState(-x, -v, -a, -j)

    /**
     * Returns the state with velocity, acceleration, and jerk zeroed.
     */
    fun stationary() = MotionState(x, 0.0, 0.0, 0.0)

    override fun toString() = String.format("(x=%.3f, v=%.3f, a=%.3f, j=%.3f)", x, v, a, j)
}
