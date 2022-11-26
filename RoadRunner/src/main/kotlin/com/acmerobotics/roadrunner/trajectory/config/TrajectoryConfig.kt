package com.acmerobotics.roadrunner.trajectory.config

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.epsilonEquals

/**
 * Configuration describing a basic trajectory (a simpler frontend alternative to [BaseTrajectoryBuilder]).
 */
data class TrajectoryConfig(
    val startPose: Pose2d,
    val startTangent: Double,
    val waypoints: List<Waypoint>,
    val resolution: Double
) {
    // the file format changes relatively frequently
    // fortunately the contents are human-readable and can be manually translated when the format changes
    // TODO: major changes will trigger a version increment in 1.0.0+
    val version = 1

    /**
     * Heading interpolation for a specific trajectory configuration step.
     */
    enum class HeadingInterpolationType {
        TANGENT,
        CONSTANT,
        LINEAR,
        SPLINE
    }

    /**
     * Description of a single segment of a composite trajectory.
     */
    data class Waypoint(
        val position: Vector2d,
        val heading: Double,
        val tangent: Double,
        val interpolationType: HeadingInterpolationType
    )

    @Suppress("ComplexMethod")
    fun toTrajectoryBuilder(groupConfig: TrajectoryGroupConfig): TrajectoryBuilder? {
        val builder = TrajectoryBuilder(
            startPose,
            startTangent,
            groupConfig.velConstraint,
            groupConfig.accelConstraint,
            resolution = resolution
        )

        for (i in waypoints.indices) {
            val (position, heading, tangent, interpolationType) = waypoints[i]

            val prevTangent = if (i == 0) {
                startTangent
            } else {
                waypoints[i - 1].tangent
            }
            val prevPosition = if (i == 0) {
                startPose.vec()
            } else {
                waypoints[i - 1].position
            }
            val interWaypointAngle = (position - prevPosition).angle()
            val line = Angle.normDelta(prevTangent - tangent) epsilonEquals 0.0 &&
                Angle.normDelta(tangent - interWaypointAngle) epsilonEquals 0.0

            when (interpolationType) {
                HeadingInterpolationType.TANGENT -> if (line) {
                    builder.lineTo(position)
                } else {
                    builder.splineTo(position, tangent)
                }
                HeadingInterpolationType.CONSTANT -> if (line) {
                    builder.lineToConstantHeading(position)
                } else {
                    builder.splineToConstantHeading(position, tangent)
                }
                HeadingInterpolationType.LINEAR -> if (line) {
                    builder.lineToLinearHeading(Pose2d(position, heading))
                } else {
                    builder.splineToLinearHeading(Pose2d(position, heading), tangent)
                }
                HeadingInterpolationType.SPLINE -> if (line) {
                    builder.lineToSplineHeading(Pose2d(position, heading))
                } else {
                    builder.splineToSplineHeading(Pose2d(position, heading), tangent)
                }
            }
        }

        return builder
    }

    fun toTrajectory(groupConfig: TrajectoryGroupConfig) = toTrajectoryBuilder(groupConfig)?.build()
}
