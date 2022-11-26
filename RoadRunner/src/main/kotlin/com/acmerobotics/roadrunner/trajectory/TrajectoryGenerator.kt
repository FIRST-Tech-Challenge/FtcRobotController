package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.*
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.util.epsilonEquals

/**
 * Trajectory generator for creating trajectories with dynamic and static constraints from paths.
 */
object TrajectoryGenerator {

    private fun generateProfile(
        path: Path,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint,
        start: MotionState,
        goal: MotionState,
        resolution: Double
    ): MotionProfile {
        return MotionProfileGenerator.generateMotionProfile(
            start,
            goal,
            { s ->
                val t = path.reparam(s)
                velocityConstraint[
                    s,
                    path[s, t],
                    path.deriv(s, t),
                    Pose2d()
                ]
            },
            { s ->
                val t = path.reparam(s)
                accelerationConstraint[
                    s,
                    path[s, t],
                    path.deriv(s, t),
                    Pose2d()
                ]
            },
            resolution
        )
    }

    private fun generateSimpleProfile(
        maxProfileVel: Double,
        maxProfileAccel: Double,
        maxProfileJerk: Double,
        start: MotionState,
        goal: MotionState
    ): MotionProfile {
        return MotionProfileGenerator.generateSimpleMotionProfile(
            start,
            goal,
            maxProfileVel,
            maxProfileAccel,
            maxProfileJerk
        )
    }

    // note: this assumes that the profile position is monotonic increasing
    private fun displacementToTime(profile: MotionProfile, s: Double): Double {
        var tLo = 0.0
        var tHi = profile.duration()
        while (!(tLo epsilonEquals tHi)) {
            val tMid = 0.5 * (tLo + tHi)
            if (profile[tMid].x > s) {
                tHi = tMid
            } else {
                tLo = tMid
            }
        }
        return 0.5 * (tLo + tHi)
    }

    private fun pointToTime(path: Path, profile: MotionProfile, point: Vector2d) =
        displacementToTime(profile, path.project(point))

    private fun convertMarkers(
        path: Path,
        profile: MotionProfile,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): List<TrajectoryMarker> {
        return temporalMarkers.map { (producer, callback) ->
            TrajectoryMarker(producer.produce(profile.duration()), callback) } +
            displacementMarkers.map { (producer, callback) ->
                TrajectoryMarker(displacementToTime(profile, producer.produce(path.length())), callback) } +
            spatialMarkers.map { (point, callback) ->
                TrajectoryMarker(pointToTime(path, profile, point), callback) }
    }

    /**
     * Generate a dynamic constraint trajectory.
     * @param path path
     * @param velocityConstraints trajectory velocity constraints
     * @param accelerationConstraints trajectory acceleration constraints
     * @param start start motion state
     * @param goal goal motion state
     * @param temporalMarkers temporal markers
     * @param spatialMarkers spatial markers
     * @param resolution dynamic profile sampling resolution
     */
    @Suppress("LongParameterList")
    @JvmOverloads
    fun generateTrajectory(
        path: Path,
        velocityConstraint: TrajectoryVelocityConstraint,
        accelerationConstraint: TrajectoryAccelerationConstraint,
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0),
        temporalMarkers: List<TemporalMarker> = emptyList(),
        displacementMarkers: List<DisplacementMarker> = emptyList(),
        spatialMarkers: List<SpatialMarker> = emptyList(),
        resolution: Double = 0.25
    ): Trajectory {
        val profile = generateProfile(path, velocityConstraint, accelerationConstraint, start, goal, resolution)
        val markers = convertMarkers(path, profile, temporalMarkers, displacementMarkers, spatialMarkers)
        return Trajectory(path, profile, markers)
    }

    /**
     * Generate a simple constraint trajectory.
     * @param path path
     * @param maxProfileVel maximum velocity
     * @param maxProfileAccel maximum acceleration
     * @param maxProfileJerk maximum jerk
     * @param start start motion state
     * @param goal goal motion state
     * @param temporalMarkers temporal markers
     * @param spatialMarkers spatial markers
     */
    @Suppress("LongParameterList")
    @JvmOverloads
    fun generateSimpleTrajectory(
        path: Path,
        maxProfileVel: Double,
        maxProfileAccel: Double,
        maxProfileJerk: Double,
        start: MotionState = MotionState(0.0, 0.0, 0.0, 0.0),
        goal: MotionState = MotionState(path.length(), 0.0, 0.0, 0.0),
        temporalMarkers: List<TemporalMarker> = emptyList(),
        displacementMarkers: List<DisplacementMarker> = emptyList(),
        spatialMarkers: List<SpatialMarker> = emptyList()
    ): Trajectory {
        val profile = generateSimpleProfile(maxProfileVel, maxProfileAccel, maxProfileJerk, start, goal)
        val markers = convertMarkers(path, profile, temporalMarkers, displacementMarkers, spatialMarkers)
        return Trajectory(path, profile, markers)
    }
}
