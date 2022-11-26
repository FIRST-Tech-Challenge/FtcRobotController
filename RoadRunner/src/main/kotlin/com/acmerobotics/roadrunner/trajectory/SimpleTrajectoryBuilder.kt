package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

private fun zeroPosition(state: MotionState) = MotionState(0.0, state.v, state.a, state.j)

/**
 * Builder for trajectories with *static* constraints.
 */
class SimpleTrajectoryBuilder private constructor(
    startPose: Pose2d?,
    startTangent: Double?,
    trajectory: Trajectory?,
    t: Double?,
    private val maxProfileVel: Double,
    private val maxProfileAccel: Double,
    private val maxProfileJerk: Double,
    private val start: MotionState
) : BaseTrajectoryBuilder<SimpleTrajectoryBuilder>(startPose, startTangent, trajectory, t) {
    /**
     * Create a builder from a start pose and motion state. This is the recommended constructor for creating
     * trajectories from rest.
     */
    @JvmOverloads constructor(
        startPose: Pose2d,
        startTangent: Double = startPose.heading,
        maxProfileVel: Double,
        maxProfileAccel: Double,
        maxProfileJerk: Double = 0.0
    ) : this(
        startPose,
        startTangent,
        null,
        null,
        maxProfileVel,
        maxProfileAccel,
        maxProfileJerk,
        MotionState(0.0, 0.0, 0.0)
    )

    constructor(
        startPose: Pose2d,
        reversed: Boolean,
        maxProfileVel: Double,
        maxProfileAccel: Double,
        maxProfileJerk: Double = 0.0
    ) : this(
        startPose,
        Angle.norm(startPose.heading + if (reversed) PI else 0.0),
        maxProfileVel,
        maxProfileAccel,
        maxProfileJerk
    )

    /**
     * Create a builder from an active trajectory. This is useful for interrupting a live trajectory and smoothly
     * transitioning to a new one.
     */
    constructor(
        trajectory: Trajectory,
        t: Double,
        maxProfileVel: Double,
        maxProfileAccel: Double,
        maxProfileJerk: Double = 0.0
    ) : this(
        null,
        null,
        trajectory,
        t,
        maxProfileVel,
        maxProfileAccel,
        maxProfileJerk,
        zeroPosition(trajectory.profile[t])
    )

    override fun buildTrajectory(
        path: Path,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0)
        return TrajectoryGenerator.generateSimpleTrajectory(
            path,
            maxProfileVel,
            maxProfileAccel,
            maxProfileJerk,
            start,
            goal,
            temporalMarkers,
            displacementMarkers,
            spatialMarkers
        )
    }
}
