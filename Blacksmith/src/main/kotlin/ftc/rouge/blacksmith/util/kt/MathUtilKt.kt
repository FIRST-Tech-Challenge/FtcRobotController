package ftc.rouge.blacksmith.util.kt

import ftc.rouge.blacksmith.units.AngleUnit
import ftc.rouge.blacksmith.units.DistanceUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue

/**
 * Simple utility function to convert from cm to inches without having to type out a
 * long function name (and without having to do `<Int>.toDouble()` in Kotlin)
 *
 * Kotlin usage examples:
 * ```
 * val inches = 30.toIn()
 * ```
 *
 * Java usage examples:
 * ```java
 * double inches = MU.toIn(30);
 * ```
 *
 * @return the number of inches in the given number of centimeters
 */
inline fun <reified T> Number.toIn(from: DistanceUnit = DistanceUnit.CM) = asNumberType<T> {
    from.toIn(this.toDouble())
}

/**
 * Simple utility function to convert from inches to cm without having to type out a
 * long function name (and without having to do `<Int>.toDouble()` in Kotlin)
 *
 * Kotlin usage examples:
 * ```
 * val cm = 30.toCm()
 * ```
 *
 * Java usage examples:
 * ```java
 * double cm = MU.toCm(30);
 * ```
 *
 * @return the number of centimeters in the given number of inches
 */
inline fun <reified T> Number.toCm(from: DistanceUnit = DistanceUnit.INCHES) = asNumberType<T> {
    from.toIn(this.toDouble()) * 2.54
}

/**
 * Simple utility function to convert from degrees to radians without having to type out a
 * long function name (and without having to do `<Int>.toDouble()` in Kotlin)
 *
 * Kotlin usage examples:
 * ```
 * val theta = 180.toRad()
 * ```
 *
 * Java usage examples:
 * ```java
 * double theta = MU.toRad(180);
 * ```
 *
 * @return the number of radians in the given number of degrees
 */
inline fun <reified T> Number.toRad(from: AngleUnit = AngleUnit.DEGREES) = asNumberType<T> {
    from.toDeg(this.toDouble()) * PI / 180
}

/**
 * Simple utility function to return a default value if the given double is NaN
 *
 * Kotlin usage examples:
 * ```
 * val doubleOk = 1.0
 * val doubleNaN = 0.0/0.0
 *
 * doubleOk.zeroIfNaN() // returns 1.0
 * doubleNaN.zeroIfNaN() // returns 0.0
 * ```
 *
 * Java usage examples:
 * ```java
 * double doubleOk = 1.0;
 * double doubleNaN = 0.0/0.0;
 *
 * MU.zeroIfNaN(doubleOk); // returns 1.0
 * MU.zeroIfNaN(doubleNaN); // returns 0.0
 * ```
 *
 * @return 'double' if it is not NaN, otherwise 'default'
 *
 * @author KG
 */
inline fun <reified T> Double.zeroIfNaN() = asNumberType<T> {
    if (isNaN()) 0.0 else this
}

/**
 * Overloaded method of the above to accept floats

 *
 * @return 'float' if it is not NaN, otherwise 'default'
 *
 * @author KG
 */
inline fun <reified T> Float.zeroIfNaN() = asNumberType<T> {
    if (isNaN()) 0.0f else this
}

/**
 * Just returns the given number if the given number is in between the given min and maxes
 * Otherwise, if `x` is less than `min`, returns `min`, and if `x` is greater than `max`,
 * returns `max`
 *
 * Kotlin usage examples:
 * ```kotlin
 * val willBe5 = 5.clamp(0, 10)
 * val willBe0 = -5.clamp(0, 10)
 * val willBe10 = 15.clamp(0, 10)
 * ```
 *
 * Java usage examples:
 * ```java
 * double willBe5 = MU.clamp(5, 0, 10);
 * double willBe0 = MU.clamp(-5, 0, 10);
 * double willBe10 = MU.clamp(15, 0, 10);
 * ```
 *
 * @param x The number to clamp
 * @param min The minimum value to return
 * @param max The maximum value to return
 *
 * @return `x` if it is in between `min` and `max`, otherwise `min` if `x` is less than `min`,
 * and `max` if `x` is greater than `max`
 */
inline fun <reified T> Number.clamp(min: Number, max: Number) = asNumberType<T> {
    this.toDouble().coerceIn(min.toDouble(), max.toDouble())
}

/**
 * Finds the average of the given numbers
 *
 * Kotlin usage examples:
 * ```kotlin
 * val average = average(1, 2, 3, 4, 5)
 * ```
 *
 * Java usage examples:
 * ```java
 * double average = MU.average(1, 2, 3, 4, 5);
 * ```
 *
 * @param xs The numbers to average
 *
 * @return The average of the given numbers
 */
inline fun <reified T> avg(vararg xs: Number) = asNumberType<T> {
    xs.sumOf { it.toDouble() } / xs.size
}

/**
 * Finds the maximum absolute value of the given numbers
 *
 * _If there are multiple numbers with the same absolute value, the first one will be returned_
 *
 * Kotlin usage examples:
 * ```kotlin
 * val maxAbs = maxAbs(1, -2, 3, -4, -5) // maxAbs == 5
 * ```
 *
 * Java usage examples:
 * ```java
 * double maxAbs = MU.maxAbs(1, -2, 3, -4, -5); // maxAbs == 5
 * ```
 *
 * @param xs The numbers to find the maximum absolute value of
 *
 * @return The maximum absolute value of the given numbers
 */
inline fun <reified T> maxMagnitude(vararg xs: Number) = asNumberType<T> {
    xs.maxByOrNull { it.toDouble().absoluteValue } ?: 0.0
}

/**
 * Returns the number unless it is within the given tolerance of the origin, in which case it
 * returns the origin.
 *
 * Kotlin usage examples:
 * ```kotlin
 * val willBe0 = withDeadzone(0.1, 0.2)
 * val willBe1 = withDeadzone(-.9, 0.2, 1.0)
 * ```
 *
 * Java usage examples:
 * ```java
 * double willBe0 = MU.withDeadzone(0.1, 0.2);
 * double willBe1 = MU.withDeadzone(0.9, 0.2, 1.0);
 * ```
 *
 * @param x The number to check
 * @param deadzone The tolerance of the origin
 * @param origin The origin to check against
 *
 * @return `x` if it is not within `deadzone` of `origin`, otherwise `origin`
 */
inline fun <reified T> Number.withDeadzone(deadzone: Number, origin: Number = 0.0) = asNumberType<T> {
    if (abs(this.toDouble() - origin.toDouble()) < abs(deadzone.toDouble())) origin.toDouble() else this.toDouble()
}

@PublishedApi
internal inline fun <reified T> asNumberType(producer: () -> Number) = when (T::class) {
    Double::class -> producer().toDouble() as T
    Float ::class -> producer().toFloat()  as T
    Long  ::class -> producer().toLong()   as T
    Int   ::class -> producer().toInt()    as T
    Short ::class -> producer().toShort()  as T
    Byte  ::class -> producer().toByte()   as T
    else -> throw IllegalArgumentException("Cannot convert to type ${T::class.simpleName}")
}
