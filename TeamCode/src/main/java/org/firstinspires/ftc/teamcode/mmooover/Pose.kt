package org.firstinspires.ftc.teamcode.mmooover

import org.firstinspires.ftc.teamcode.hardware.MotorSet
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

fun wrapAngle(angle: Double): Double {
    return when {
        angle > Math.PI -> angle - 2 * Math.PI
        angle <= -Math.PI -> angle + 2 * Math.PI
        else -> angle
    }
}

data class Pose(
    @get:JvmName("x") val x: Double,
    @get:JvmName("y") val y: Double,
    @get:JvmName("heading") val heading: Double,
) {
    companion object {
        @JvmField
        val ORIGIN = Pose(0, 0, 0)
    }

    /**
     * Vector addition (i.e. [x1, y1, heading1] + [x2, y2, heading2] = [x1 + x2, y1 + y2, heading1 + heading2])
     * Adds each component.
     */
    @JvmName("add")
    operator fun plus(other: Pose) = Pose(x + other.x, y + other.y, heading + other.heading)

    /**
     * Computes the linear (euclidean) distance between two poses, not considering
     * the headings of the poses.
     */
    fun linearDistanceTo(other: Pose): Double {
        val dx = other.x - x
        val dy = other.y - y
        return hypot(dx, dy) // this does sqrt(x**2 + y**2)
    }

    /**
     * Computes the difference between the angles of `this` Pose and the `other` Pose.
     * Wraps the result into the -PI to +PI range.
     * @return angle, radians on (-PI, PI]
     */
    fun subtractAngle(other: Pose): Double {
        return wrapAngle(other.heading - heading)
    }

    /**
     * Computes the motion required to travel to the `other` pose from `this` pose.
     */
    fun to(other: Pose, robot: TriOdoProvider): Motion {
        val dh = wrapAngle(other.heading - heading)
        val dx = other.x - x
        val dy = other.y - y
        val forward = cos(heading) * dx + sin(heading) * dy
        val strafe = sin(heading) * dx - cos(heading) * dy
        return Motion(forward, strafe, -dh * robot.forwardOffset)
    }

    /**
     * Constructor supporting non-Double types.
     */
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

    /**
     * Constructor supporting non-Double types.
     */
    constructor(forward: Number, right: Number, turn: Number) : this(
        forward.toDouble(),
        right.toDouble(),
        turn.toDouble()
    )

    var lastFL: Double = 0.0; private set
    var lastFR: Double = 0.0; private set
    var lastBL: Double = 0.0; private set
    var lastBR: Double = 0.0; private set

    fun getPowerDifferential(): Double {
        return (lastFR + lastBR) - (lastFL + lastBL)
    }

    /**
     * Apply this Motion to a MotorSet.
     * @param motors target set of motors
     * @param calibration motor bias calibration. configures how much power is used for e.g. strafing vs forwarding
     * @param factor overall speed factor. applied after all other calculations.
     */
    @JvmOverloads
    fun apply(motors: MotorSet, calibration: Calibrate, factor: Number = 1.0, s2p: Speed2Power) {
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
        fl = s2p.speed2power(fl)
        fr = s2p.speed2power(fr)
        bl = s2p.speed2power(bl)
        br = s2p.speed2power(br)
        lastFL = fl; lastFR = fr; lastBL = bl; lastBR = br
        motors.set(fl, fr, bl, br);
    }
}