package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
import kotlin.math.abs
import kotlin.math.max

/**
 * Constraint limiting angular velocity.
 */
class AngularVelocityConstraint(
    private val maxAngularVel: Double
) : TrajectoryVelocityConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d): Double {
        val omega0 = baseRobotVel.heading
        if (abs(omega0) >= maxAngularVel) {
            throw UnsatisfiableConstraint()
        }

        return max(
            (maxAngularVel - omega0) / deriv.heading,
            (-maxAngularVel - omega0) / deriv.heading
        )
    }
}
