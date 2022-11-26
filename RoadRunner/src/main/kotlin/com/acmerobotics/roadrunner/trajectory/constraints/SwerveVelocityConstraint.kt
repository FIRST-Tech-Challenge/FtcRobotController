package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.SwerveKinematics
import kotlin.math.abs
import kotlin.math.max

/**
 * Mecanum-specific drive constraints that also limit maximum wheel velocities.
 *
 * @param maxWheelVel maximum wheel velocity
 * @param trackWidth track width
 * @param wheelBase wheel base
 */
open class SwerveVelocityConstraint @JvmOverloads constructor(
    private val maxWheelVel: Double,
    private val trackWidth: Double,
    private val wheelBase: Double = trackWidth
) : TrajectoryVelocityConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d): Double {
        val wheel0 = SwerveKinematics.robotToWheelVelocities(baseRobotVel, trackWidth, wheelBase)
        if (wheel0.map(::abs).maxOrNull()!! >= maxWheelVel) {
            throw UnsatisfiableConstraint()
        }

        val robotDeriv = Kinematics.fieldToRobotVelocity(pose, deriv)

        val wheel = SwerveKinematics.robotToWheelVelocities(robotDeriv, trackWidth, wheelBase)
        return wheel0.zip(wheel).map {
            max(
                (maxWheelVel - it.first) / it.second,
                (-maxWheelVel - it.first) / it.second
            )
        }.minOrNull()!!
    }
}
