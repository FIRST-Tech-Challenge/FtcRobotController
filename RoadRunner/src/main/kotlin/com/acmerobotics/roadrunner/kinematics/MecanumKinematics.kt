package com.acmerobotics.roadrunner.kinematics

import com.acmerobotics.roadrunner.geometry.Pose2d

/**
 * Mecanum drive kinematic equations. All wheel positions and velocities are given starting with front left and
 * proceeding counter-clockwise (i.e., front left, rear left, rear right, front right). Robot poses are specified in a
 * coordinate system with positive x pointing forward, positive y pointing left, and positive heading measured
 * counter-clockwise from the x-axis.
 *
 * [This paper](http://www.chiefdelphi.com/media/papers/download/2722) provides a motivated derivation.
 */
object MecanumKinematics {

    /**
     * Computes the wheel velocities corresponding to [robotVel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @param lateralMultiplier multiplicative gain to adjust for systematic, proportional lateral error (gain greater
     * than 1.0 corresponds to overcompensation).
     */
    @JvmStatic
    @JvmOverloads
    fun robotToWheelVelocities(
        robotVel: Pose2d,
        trackWidth: Double,
        wheelBase: Double = trackWidth,
        lateralMultiplier: Double = 1.0
    ): List<Double> {
        val k = (trackWidth + wheelBase) / 2.0
        return listOf(
            robotVel.x - lateralMultiplier * robotVel.y - k * robotVel.heading,
            robotVel.x + lateralMultiplier * robotVel.y - k * robotVel.heading,
            robotVel.x - lateralMultiplier * robotVel.y + k * robotVel.heading,
            robotVel.x + lateralMultiplier * robotVel.y + k * robotVel.heading
        )
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given the provided [trackWidth] and
     * [wheelBase].
     *
     * @param robotAccel acceleration of the robot in its reference frame
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @param lateralMultiplier multiplicative gain to adjust for systematic, proportional lateral error (gain greater
     * than 1.0 corresponds to overcompensation).
     */
    @JvmStatic
    @JvmOverloads
    // follows from linearity of the derivative
    fun robotToWheelAccelerations(
        robotAccel: Pose2d,
        trackWidth: Double,
        wheelBase: Double = trackWidth,
        lateralMultiplier: Double = 1.0
    ) =
        robotToWheelVelocities(
            robotAccel,
            trackWidth,
            wheelBase,
            lateralMultiplier
        )

    /**
     * Computes the robot velocity corresponding to [wheelVelocities] and the given drive parameters.
     *
     * @param wheelVelocities wheel velocities (or wheel position deltas)
     * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
     * @param wheelBase distance between pairs of wheels on the same side of the robot
     * @param lateralMultiplier multiplicative gain to adjust for systematic, proportional lateral error (gain greater
     * than 1.0 corresponds to overcompensation).
     */
    @JvmStatic
    @JvmOverloads
    fun wheelToRobotVelocities(
        wheelVelocities: List<Double>,
        trackWidth: Double,
        wheelBase: Double = trackWidth,
        lateralMultiplier: Double = 1.0
    ): Pose2d {
        val k = (trackWidth + wheelBase) / 2.0
        val (frontLeft, rearLeft, rearRight, frontRight) = wheelVelocities
        return Pose2d(
            wheelVelocities.sum(),
            (rearLeft + frontRight - frontLeft - rearRight) / lateralMultiplier,
            (rearRight + frontRight - frontLeft - rearLeft) / k
        ) * 0.25
    }
}
