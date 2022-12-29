@file:JvmName("MU")

package ftc.rogue.blacksmith.util

import ftc.rogue.blacksmith.units.AngleUnit
import ftc.rogue.blacksmith.units.DistanceUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue

@JvmOverloads
fun Number.toIn(from: DistanceUnit = DistanceUnit.CM): Double = from.toIn(this.toDouble())

fun Number.toCm(from: DistanceUnit = DistanceUnit.INCHES): Double = from.toIn(this.toDouble()) * 2.54

fun Number.toRad(from: AngleUnit = AngleUnit.DEGREES): Double = from.toDeg(this.toDouble()) * PI / 180


fun Double.zeroIfNaN() = if (isNaN()) 0.0 else this

fun Float.zeroIfNaN() = if (isNaN()) 0.0f else this


fun Number.isInRange(min: Number, max: Number) =
    this.toDouble() in min.toDouble()..max.toDouble()


fun Number.clamp(min: Number, max: Number) =
    this.toDouble().coerceIn(min.toDouble(), max.toDouble())


fun avg(vararg xs: Number) = xs.sumOf { it.toDouble() } / xs.size


fun maxMagnitude(vararg xs: Number) = xs.maxByOrNull { it.toDouble().absoluteValue } ?: 0.0

fun maxMagnitudeAbs(vararg xs: Number) = xs.maxOfOrNull { it.toDouble().absoluteValue } ?: 0.0


@JvmOverloads
fun Number.withDeadzone(deadzone: Number, origin: Number = 0.0) =
    if (abs(this.toDouble() - origin.toDouble()) < abs(deadzone.toDouble())) origin.toDouble() else this.toDouble()
