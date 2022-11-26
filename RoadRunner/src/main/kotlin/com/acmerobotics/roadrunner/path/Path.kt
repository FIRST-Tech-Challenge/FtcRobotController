package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.epsilonEquals
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * Path composed of a list of parametric curves and heading interpolators.
 *
 * @param segments list of path segments
 */
class Path(val segments: List<PathSegment>) {

    /**
     * @param segment single path segment
     */
    constructor(segment: PathSegment) : this(listOf(segment))

    /**
     * Returns the length of the path.
     */
    fun length() = segments.sumByDouble { it.length() }

    fun segment(s: Double): Pair<PathSegment, Double> {
        if (s <= 0.0) {
            return segments.first() to 0.0
        }
        var remainingDisplacement = s
        for (segment in segments) {
            if (remainingDisplacement <= segment.length()) {
                return segment to remainingDisplacement
            }
            remainingDisplacement -= segment.length()
        }
        return segments.last() to segments.last().length()
    }

    /**
     * Returns the pose [s] units along the path.
     */
    @JvmOverloads
    operator fun get(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment[remainingDisplacement, t]
    }

    /**
     * Returns the pose derivative [s] units along the path.
     */
    @JvmOverloads
    fun deriv(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment.deriv(remainingDisplacement, t)
    }

    /**
     * Returns the pose second derivative [s] units along the path.
     */
    @JvmOverloads
    fun secondDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment.secondDeriv(remainingDisplacement, t)
    }

    @JvmOverloads
    internal fun internalDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment.internalDeriv(remainingDisplacement, t)
    }

    @JvmOverloads
    internal fun internalSecondDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        val (segment, remainingDisplacement) = segment(s)
        return segment.internalSecondDeriv(remainingDisplacement, t)
    }

    internal fun reparam(s: Double): Double {
        val (segment, remainingDisplacement) = segment(s)
        return segment.reparam(remainingDisplacement)
    }

    /**
     * Project [queryPoint] onto the current path using the iterative method described
     * [here](http://www.geometrie.tugraz.at/wallner/sproj.pdf).
     *
     * @param queryPoint query queryPoint
     * @param projectGuess guess for the projected queryPoint's s along the path
     */
    fun fastProject(queryPoint: Vector2d, projectGuess: Double = length() / 2.0): Double {
        // we use the first-order method (since we already compute the arc length param
        var s = projectGuess
        repeat(200) {
            val t = reparam(s)
            val pathPoint = get(s, t).vec()
            val deriv = deriv(s, t).vec()

            val ds = (queryPoint - pathPoint) dot deriv

            if (ds epsilonEquals 0.0) {
                return@repeat
            }

            s += ds

            if (s <= 0.0) {
                return@repeat
            }

            if (s >= length()) {
                return@repeat
            }
        }

        return max(0.0, min(s, length()))
    }

    /**
     * Project [queryPoint] onto the current path by applying [fastProject] with various
     * guesses along the path.
     *
     * @param queryPoint query queryPoint
     * @param ds spacing between guesses
     */
    fun project(queryPoint: Vector2d, ds: Double = 0.25): Double {
        val samples = (length() / ds).roundToInt()

        val guesses = DoubleProgression.fromClosedInterval(0.0, length(), samples)

        val results = guesses.map { fastProject(queryPoint, it) }

        return results.minByOrNull { this[it].vec().distTo(queryPoint) } ?: 0.0
    }

    /**
     * Returns the start pose.
     */
    fun start() = get(0.0)

    /**
     * Returns the start pose derivative.
     */
    fun startDeriv() = deriv(0.0)

    /**
     * Returns the start pose second derivative.
     */
    fun startSecondDeriv() = secondDeriv(0.0)

    /**
     * Returns the end pose.
     */
    fun end() = get(length())

    /**
     * Returns the end pose derivative.
     */
    fun endDeriv() = deriv(length())

    /**
     * Returns the end pose second derivative.
     */
    fun endSecondDeriv() = secondDeriv(length())
}
