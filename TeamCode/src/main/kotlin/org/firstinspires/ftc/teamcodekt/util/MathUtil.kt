@file:JvmName("MU")

package org.firstinspires.ftc.teamcodekt.util

/**
 * Simple utility function to convert from cm to inces without having to type out a
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
 */
fun Number.toIn() = this.toDouble() / 2.54

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
 */
fun Number.toRad() = Math.toRadians(this.toDouble())

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
 * @param default The default value to return if 'double' is NaN
 *
 * @return 'double' if it is not NaN, otherwise 'default'
 *
 * @author KG
 */
fun Double.zeroIfNaN() = if (isNaN()) 0.0 else this

/**
 * Overloaded method of the above to accept floats
 *
 * @param default The degree value to convert
 *
 * @return 'float' if it is not NaN, otherwise 'default'
 *
 * @author KG
 */
fun Float.zeroIfNaN() = if (isNaN()) 0.0f else this

/**
 * Just returns `true` if the given number is in between the given min and maxes
 *
 * Kotlin usage examples:
 * ```kotlin
 * val willBeTrue = 5.inRange(0, 10)
 * ```
 *
 * Java usage examples:
 * ```java
 * boolean willBeTrue = MU.inRange(5, 0, 10);
 * ```
 */
fun Number.isInRange(min: Number, max: Number) =
    this.toDouble() in min.toDouble()..max.toDouble()

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
fun Number.clamp(min: Number, max: Number) =
    this.toDouble().coerceIn(min.toDouble(), max.toDouble())

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
 */
fun avg(vararg xs: Number) = xs.sumOf { it.toDouble() } / xs.size
