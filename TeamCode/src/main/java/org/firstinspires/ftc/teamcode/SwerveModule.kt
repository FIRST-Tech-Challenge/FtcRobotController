package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.clampf
import kotlin.math.floor
import kotlin.math.sign

class SwerveModule(val topMotor: DcMotor, val bottomMotor: DcMotor, private val ppr: Double, private val totalRatio: Double) {
    private var targetAngleTicks: Int = getAngleTicks()
    var power: Double = 0.0

    fun getAngle(): Double {
        return (getAngleTicks() / ppr) * totalRatio
    }

    fun rotate(delta: Double) {
        targetAngleTicks = angleToTicks(delta) + getAngleTicks()
    }

    fun setAngle(angle: Double) {
        val angleDifference: Double = angleDifference(angle, getAngle())
        targetAngleTicks = angleToTicks(angleDifference + getAngle())
    }

    private fun getAngleTicks(): Int {
        return (topMotor.currentPosition + bottomMotor.currentPosition) / 2
    }

    private fun angleToTicks(angle: Double): Int {
        return ((angle / ppr) * totalRatio).toInt()
    }

    private fun getRotationCount(): Int {
        return floor(getAngle()).toInt()
    }

    private fun angleDifference(angle1: Double, angle2: Double): Double {
        val diff: Double = (angle2 - angle1 + 0.5) - getRotationCount() - 180

        return if (diff < -180) {
            diff + 360
        } else {
            diff
        }
    }

    fun update() {
        val angleTicksDelta: Int = targetAngleTicks - getAngleTicks()
        topMotor.power = angleTicksDelta.sign * 0.5 + clampf(power, 0.0, 1.0) * 0.5
        bottomMotor.power = angleTicksDelta.sign * 0.5 - clampf(power, 0.0, 1.0) * 0.5
    }
}