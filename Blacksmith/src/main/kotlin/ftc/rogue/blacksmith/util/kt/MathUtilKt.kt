package ftc.rogue.blacksmith.util.kt

import ftc.rogue.blacksmith.units.AngleUnit
import ftc.rogue.blacksmith.units.DistanceUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.pow

inline fun <reified T : Number> Number.toIn(from: DistanceUnit = DistanceUnit.CM) = asNumberType<T> {
    from.toIn(this.toDouble())
}

inline fun <reified T : Number> Number.toCm(from: DistanceUnit = DistanceUnit.INCHES) = asNumberType<T> {
    from.toIn(this.toDouble()) * 2.54
}

inline fun <reified T : Number> Number.toRad(from: AngleUnit = AngleUnit.DEGREES) = asNumberType<T> {
    from.toDeg(this.toDouble()) * PI / 180
}


inline fun <reified T : Number> Number.clamp(min: Number, max: Number) = asNumberType<T> {
    this.toDouble().coerceIn(min.toDouble(), max.toDouble())
}


inline fun <reified T : Number> avg(vararg xs: Number) = asNumberType<T> {
    xs.sumOf { it.toDouble() } / xs.size
}


inline fun <reified T : Number> maxMagnitude(vararg xs: Number) = asNumberType<T> {
    xs.maxByOrNull { it.toDouble().absoluteValue } ?: 0.0
}

inline fun <reified T : Number> maxMagnitudeAbs(vararg xs: Number) = asNumberType<T> {
    xs.maxOfOrNull { it.toDouble().absoluteValue } ?: 0.0
}


inline fun <reified T : Number> Number.withDeadzone(deadzone: Number, origin: Number = 0.0) = asNumberType<T> {
    if (abs(this.toDouble() - origin.toDouble()) < abs(deadzone.toDouble())) origin.toDouble() else this.toDouble()
}


@JvmSynthetic
infix fun Number.pow(exponent: Number) = this.toDouble().pow(exponent.toDouble())

@PublishedApi
internal inline fun <reified T : Number> asNumberType(producer: () -> Number) = when (T::class) {
    Double::class -> producer().toDouble() as T
    Float ::class -> producer().toFloat()  as T
    Long  ::class -> producer().toLong()   as T
    Int   ::class -> producer().toInt()    as T
    Short ::class -> producer().toShort()  as T
    Byte  ::class -> producer().toByte()   as T
    else -> throw IllegalArgumentException("Cannot convert number to type ${T::class.simpleName}")
}
