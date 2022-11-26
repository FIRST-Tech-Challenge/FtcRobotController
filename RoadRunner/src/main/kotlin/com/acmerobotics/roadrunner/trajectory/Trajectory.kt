package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionProfile

/**
 * Trajectory backed by a [Path] and a [MotionProfile].
 *
 * @param path path
 * @param profile motion profile
 */
class Trajectory @JvmOverloads constructor(
    val path: Path,
    val profile: MotionProfile,
    val markers: List<TrajectoryMarker> = emptyList()
) {
    fun duration() = profile.duration()

    operator fun get(time: Double) = path[profile[time].x]

    fun velocity(time: Double): Pose2d {
        val motionState = profile[time]
        return path.deriv(motionState.x) * motionState.v
    }

    fun acceleration(time: Double): Pose2d {
        val motionState = profile[time]
        return path.secondDeriv(motionState.x) * motionState.v * motionState.v +
            path.deriv(motionState.x) * motionState.a
    }

    fun start() = path[0.0, 0.0]

    fun end() = path[path.length(), 1.0]
}
