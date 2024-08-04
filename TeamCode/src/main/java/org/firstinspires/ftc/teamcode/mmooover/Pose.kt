package org.firstinspires.ftc.teamcode.mmooover

import org.firstinspires.ftc.teamcode.hardware.MotorSet
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

private fun wrapAngle(angle: Double): Double {
    return when {
        angle > Math.PI -> angle - 2 * Math.PI
        angle < -Math.PI -> angle + 2 * Math.PI
        else -> angle
    }
}

data class Pose(
    @get:JvmName("x") val x: Double,
    @get:JvmName("y") val y: Double,
    @get:JvmName("heading") val heading: Double,
) {
    /**
     * vector addition (i.e. [x1, y1, heading1] + [x2, y2, heading2] = [x1 + x2, y1 + y2, heading1 + heading2])
     */
    @JvmName("add")
    operator fun plus(other: Pose) = Pose(x + other.x, y + other.y, heading + other.heading)

    fun linearDistanceTo(other: Pose): Double {
        val dx = other.x - x
        val dy = other.y - y
        return hypot(dx, dy)
    }
    fun subtractAngle(other: Pose): Double {
        return wrapAngle(other.heading - heading)
    }

    fun to(other: Pose, robot: TriOdoProvider): Motion {
        val dh = wrapAngle(other.heading - heading)
        val dx = other.x - x
        val dy = other.y - y
        val forward = cos(heading) * dx + sin(heading) * dy
        val strafe = sin(heading) * dx - cos(heading) * dy
        return Motion(forward, strafe, -dh * robot.forwardOffset)
    }

    @JvmOverloads
    constructor(x: Number, y: Number, heading: Number = 0.0) : this(
        x.toDouble(),
        y.toDouble(),
        heading.toDouble()
    )
}

data class Motion(
    @get:JvmName("forward") val forward: Double,
    @get:JvmName("right") val right: Double,
    @get:JvmName("turn") val turn: Double
) {
    data class Calibrate(val preferForward: Double, val preferStrafe: Double, val preferTurn: Double)

    constructor(forward: Number, right: Number, turn: Number) : this(
        forward.toDouble(),
        right.toDouble(),
        turn.toDouble()
    )

    @JvmOverloads
    fun apply(motors: MotorSet, calibration: Calibrate, factor: Number = 1.0) {
        val fwBias = calibration.preferForward
        val rtBias = calibration.preferStrafe
        val turnBias = calibration.preferTurn
        var fl = forward * fwBias + right * rtBias - turn * turnBias
        var fr = forward * fwBias - right * rtBias + turn * turnBias
        var bl = forward * fwBias - right * rtBias - turn * turnBias
        var br = forward * fwBias + right * rtBias + turn * turnBias
        val div = max(1.0, max(abs(fl), max(abs(fr), max(abs(bl), abs(br)))))
        fl /= div
        fr /= div
        bl /= div
        br /= div
        val factorD = factor.toDouble()
        fl *= factorD
        fr *= factorD
        bl *= factorD
        br *= factorD
        motors.set(fl, fr, bl, br);
    }
}