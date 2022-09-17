package org.firstinspires.ftc.teamcode.util.math

import org.firstinspires.ftc.teamcode.util.math.MathUtil.degrees
import kotlin.math.* // ktlint-disable no-wildcard-imports

data class Angle(
    var angle: Double,
    var unit: AngleUnit
) {

    val deg: Double get() = angle.degrees

    val cos = cos(angle)
    val sin = sin(angle)
    val abs = angle.absoluteValue
    val copy get() = Angle(angle, unit)

    fun wrap(): Angle {
        var heading = angle
        while (heading < -PI)
            heading += 2 * PI
        while (heading > PI)
            heading -= 2 * PI
        return Angle(heading, unit)
    }

    operator fun plus(other: Angle) = when (unit) {
        AngleUnit.RAD -> Angle(angle + other.angle, unit).wrap()
        AngleUnit.RAW -> Angle(angle + other.angle, unit)
    }

    operator fun minus(other: Angle) = plus(other.unaryMinus())

    operator fun unaryMinus() = when (unit) {
        AngleUnit.RAD -> Angle(-angle, unit).wrap()
        AngleUnit.RAW -> Angle(-angle, unit)
    }

    operator fun times(scalar: Double) = Angle(angle * scalar, unit)
    operator fun div(scalar: Double) = Angle(angle / scalar, unit)

    companion object {
        val EAST get() = Angle(0.0, AngleUnit.RAD)
    }
}
