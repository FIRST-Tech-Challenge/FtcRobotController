package com.acmerobotics.roadrunner.util

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Various math utilities.
 */
object MathUtil {

    /**
     * Returns the real solutions to the quadratic ax^2 + bx + c.
     */
    @JvmStatic
    fun solveQuadratic(a: Double, b: Double, c: Double): List<Double> {
        val disc = b * b - 4 * a * c
        return when {
            disc epsilonEquals 0.0 -> listOf(-b / (2 * a))
            disc > 0.0 -> listOf(
                (-b + sqrt(disc)) / (2 * a),
                (-b - sqrt(disc)) / (2 * a)
            )
            else -> emptyList()
        }
    }
}

const val EPSILON = 1e-6

infix fun Double.epsilonEquals(other: Double) = abs(this - other) < EPSILON
