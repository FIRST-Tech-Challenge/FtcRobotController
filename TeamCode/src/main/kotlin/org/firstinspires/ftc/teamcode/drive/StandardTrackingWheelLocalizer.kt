package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class StandardTrackingWheelLocalizer(
    hardware: HardwareManager,
    private val lastEncPositions: MutableList<Int>,
    private val lastEncVels: MutableList<Int>
) :
    ThreeTrackingWheelLocalizer(
        listOf(
            Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // left
            Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0),  // right
            Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
        )
    ) {
    private val leftEncoder = hardware.leftEncoder!!
    private val rightEncoder = hardware.rightEncoder!!
    private val rearEncoder = hardware.rearEncoder!!

    private var xMultiplier = 0.97637657
    private var yMultiplier = 1.02772238

    override fun getWheelPositions(): List<Double> {
        val leftPos = leftEncoder.currentPosition
        val rightPos = rightEncoder.currentPosition
        val frontPos = rearEncoder.currentPosition

        lastEncPositions.clear()
        lastEncPositions.add(leftPos.toInt())
        lastEncPositions.add(rightPos.toInt())
        lastEncPositions.add(frontPos.toInt())

        return listOf(
            encoderTicksToInches(leftPos * xMultiplier),
            encoderTicksToInches(rightPos * xMultiplier),
            encoderTicksToInches(frontPos * yMultiplier)
        )
    }

    override fun getWheelVelocities(): List<Double> {
        val leftVel = leftEncoder.correctedVelocity.toInt()
        val rightVel = rightEncoder.correctedVelocity.toInt()
        val frontVel = rearEncoder.correctedVelocity.toInt()

        lastEncVels.clear()
        lastEncVels.add(leftVel)
        lastEncVels.add(rightVel)
        lastEncVels.add(frontVel)

        return listOf(
            encoderTicksToInches(leftVel.toDouble() * xMultiplier),
            encoderTicksToInches(rightVel.toDouble() * xMultiplier),
            encoderTicksToInches(frontVel.toDouble() * yMultiplier)
        )
    }

    companion object {
        var TICKS_PER_REV: Double = 2000.0
        var WHEEL_RADIUS: Double = 0.6299213 // in
        var GEAR_RATIO: Double = 1.0 // output (wheel) speed / input (encoder) speed

        var LATERAL_DISTANCE: Double = 8.6918432 // in; distance between the left and right wheels
        var FORWARD_OFFSET: Double = -6.625 // in; offset of the lateral wheel

        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}