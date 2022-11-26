package com.acmerobotics.roadrunner.kinematics

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

/**
 * A collection of methods for various kinematics-related tasks.
 */
object Kinematics {

    /**
     * Returns the robot pose velocity corresponding to [fieldPose] and [fieldVel].
     */
    @JvmStatic
    fun fieldToRobotVelocity(fieldPose: Pose2d, fieldVel: Pose2d) =
        Pose2d(fieldVel.vec().rotated(-fieldPose.heading), fieldVel.heading)

    /**
     * Returns the robot pose acceleration corresponding to [fieldPose], [fieldVel], and [fieldAccel].
     */
    @JvmStatic
    fun fieldToRobotAcceleration(fieldPose: Pose2d, fieldVel: Pose2d, fieldAccel: Pose2d) =
        Pose2d(
            fieldAccel.vec().rotated(-fieldPose.heading),
            fieldAccel.heading
        ) +
        Pose2d(
            -fieldVel.x * sin(fieldPose.heading) + fieldVel.y * cos(fieldPose.heading),
            -fieldVel.x * cos(fieldPose.heading) - fieldVel.y * sin(fieldPose.heading),
            0.0
        ) * fieldVel.heading

    /**
     * Returns the error between [targetFieldPose] and [currentFieldPose] in the field frame.
     */
    @JvmStatic
    fun calculateFieldPoseError(targetFieldPose: Pose2d, currentFieldPose: Pose2d) =
        Pose2d(
            (targetFieldPose - currentFieldPose).vec(),
            Angle.normDelta(targetFieldPose.heading - currentFieldPose.heading)
        )

    /**
     * Returns the error between [targetFieldPose] and [currentFieldPose] in the robot frame.
     */
    @JvmStatic
    fun calculateRobotPoseError(targetFieldPose: Pose2d, currentFieldPose: Pose2d): Pose2d {
        val errorInFieldFrame = calculateFieldPoseError(targetFieldPose, currentFieldPose)
        return Pose2d(
            errorInFieldFrame.vec().rotated(-currentFieldPose.heading),
            errorInFieldFrame.heading
        )
    }

    /**
     * Computes the motor feedforward (i.e., open loop powers) for the given set of coefficients.
     */
    @JvmStatic
    fun calculateMotorFeedforward(
        vels: List<Double>,
        accels: List<Double>,
        kV: Double,
        kA: Double,
        kStatic: Double
    ) =
        vels.zip(accels)
            .map { (vel, accel) -> calculateMotorFeedforward(vel, accel, kV, kA, kStatic) }

    /**
     * Computes the motor feedforward (i.e., open loop power) for the given set of coefficients.
     */
    @JvmStatic
    fun calculateMotorFeedforward(vel: Double, accel: Double, kV: Double, kA: Double, kStatic: Double): Double {
        val basePower = vel * kV + accel * kA
        return if (basePower epsilonEquals 0.0) {
            0.0
        } else {
            basePower + sign(basePower) * kStatic
        }
    }

    /**
     * Performs a relative odometry update. Note: this assumes that the robot moves with constant velocity over the
     * measurement interval.
     */
    @JvmStatic
    fun relativeOdometryUpdate(fieldPose: Pose2d, robotPoseDelta: Pose2d): Pose2d {
        val dtheta = robotPoseDelta.heading
        val (sineTerm, cosTerm) = if (dtheta epsilonEquals 0.0) {
            1.0 - dtheta * dtheta / 6.0 to dtheta / 2.0
        } else {
            sin(dtheta) / dtheta to (1 - cos(dtheta)) / dtheta
        }

        val fieldPositionDelta = Vector2d(
            sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
            cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
        )

        val fieldPoseDelta = Pose2d(fieldPositionDelta.rotated(fieldPose.heading), robotPoseDelta.heading)

        return Pose2d(
            fieldPose.x + fieldPoseDelta.x,
            fieldPose.y + fieldPoseDelta.y,
            Angle.norm(fieldPose.heading + fieldPoseDelta.heading)
        )
    }
}
