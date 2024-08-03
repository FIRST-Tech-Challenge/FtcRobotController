package org.firstinspires.ftc.teamcode.localization

import kotlin.contracts.contract
import kotlin.math.cos
import kotlin.math.sin

data class Pose(
    @get:JvmName("x") val x: Double,
    @get:JvmName("y") val y: Double,
    @get:JvmName("heading") val heading: Double,
) {
    companion object {
        const val EPS = 1e-9
    }

    /**
     * vector addition (i.e. [x1, y1, heading1] + [x2, y2, heading2] = [x1 + x2, y1 + y2, heading1 + heading2])
     */
    @JvmName("add")
    operator fun plus(other: Pose) = Pose(x + other.x, y + other.y, heading + other.heading)

    fun to(other: Pose): Motion {
        if (other.heading - heading > EPS) TODO("Not implemented")
        val dx = other.x - x
        val dy = other.y - y
        val forward = cos(heading) * dx + sin(heading) * dy
        val strafe = sin(heading) * dx - cos(heading) * dy
        return Motion(forward, strafe, 0)
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
    constructor(forward: Number, right: Number, turn: Number) : this(
        forward.toDouble(),
        right.toDouble(),
        turn.toDouble()
    )

    fun apply(base: Pose): Pose {
        TODO()
    }
}