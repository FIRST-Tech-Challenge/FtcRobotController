package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d

/**
 * Composite constraint representing the minimum of its constituent velocity constraints.
 */
class MinVelocityConstraint(
    private val constraints: List<TrajectoryVelocityConstraint>
) : TrajectoryVelocityConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d) =
        constraints.map { it[s, pose, deriv, baseRobotVel] }.minOrNull()!!
}
