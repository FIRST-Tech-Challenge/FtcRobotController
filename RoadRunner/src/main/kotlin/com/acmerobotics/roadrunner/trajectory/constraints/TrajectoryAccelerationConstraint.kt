package com.acmerobotics.roadrunner.trajectory.constraints

import com.acmerobotics.roadrunner.geometry.Pose2d

/**
 * Motion profile acceleration constraint.
 */
fun interface TrajectoryAccelerationConstraint {

    /**
     * Returns the maximum profile acceleration.
     *
     * @param s path displacement
     * @param pose pose
     * @param deriv pose derivative
     * @param baseRobotVel additive base velocity
     */
    operator fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d): Double
}
