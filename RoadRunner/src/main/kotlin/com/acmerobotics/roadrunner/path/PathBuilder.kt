package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

/**
 * Exception thrown by [PathBuilder].
 */
abstract class PathBuilderException : RuntimeException()

/**
 * Exception thrown when [PathBuilder] methods are chained illegally. This commonly arises when switching from
 * non-tangent interpolation back to tangent interpolation and when splicing paths.
 */
class PathContinuityViolationException : PathBuilderException()

/**
 * Exception thrown when empty path segments are requested.
 */
class EmptyPathSegmentException : PathBuilderException()

/**
 * Easy-to-use builder for creating [Path] instances.
 *
 * @param startPose start pose
 * @param startTangent start tangent
 * @param path previous path
 * @param s displacement in previous path
 */
class PathBuilder private constructor(
    startPose: Pose2d?,
    startTangent: Double?,
    internal val path: Path?,
    internal val s: Double?
) {
    @JvmOverloads
    constructor(startPose: Pose2d, startTangent: Double = startPose.heading) :
        this(startPose, startTangent, null, null)

    constructor(startPose: Pose2d, reversed: Boolean) :
        this(startPose, Angle.norm(startPose.heading + if (reversed) PI else 0.0))

    constructor(path: Path, s: Double) : this(null, null, path, s)

    private var currentPose: Pose2d? = startPose
    private var currentTangent: Double? = startTangent

    private var segments = mutableListOf<PathSegment>()

    private fun makeLine(end: Vector2d): LineSegment {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        if (start.vec() epsilonEquals end) {
            throw EmptyPathSegmentException()
        }

        return LineSegment(start.vec(), end)
    }

    private fun makeSpline(endPosition: Vector2d, endTangent: Double): QuinticSpline {
        val startPose = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        if (startPose.vec() epsilonEquals endPosition) {
            throw EmptyPathSegmentException()
        }

        val derivMag = (startPose.vec() distTo endPosition)
        val (startWaypoint, endWaypoint) = if (currentPose == null) {
            val startDeriv = path!!.internalDeriv(s!!).vec()
            val startSecondDeriv = path.internalSecondDeriv(s).vec()
            QuinticSpline.Knot(startPose.vec(), startDeriv, startSecondDeriv) to
                QuinticSpline.Knot(endPosition, Vector2d.polar(derivMag, endTangent))
        } else {
            QuinticSpline.Knot(startPose.vec(), Vector2d.polar(derivMag, currentTangent!!)) to
                QuinticSpline.Knot(endPosition, Vector2d.polar(derivMag, endTangent))
        }

        return QuinticSpline(startWaypoint, endWaypoint)
    }

    private fun makeTangentInterpolator(curve: ParametricCurve): TangentInterpolator {
        if (currentPose == null) {
            val prevInterpolator = path!!.segment(s!!).first.interpolator
            if (prevInterpolator !is TangentInterpolator) {
                throw PathContinuityViolationException()
            }
            return TangentInterpolator(prevInterpolator.offset)
        }

        val startHeading = curve.tangentAngle(0.0, 0.0)

        val interpolator = TangentInterpolator(currentPose!!.heading - startHeading)
        interpolator.init(curve)
        return interpolator
    }

    private fun makeConstantInterpolator(): ConstantInterpolator {
        val currentHeading = currentPose?.heading ?: throw PathContinuityViolationException()

        return ConstantInterpolator(currentHeading)
    }

    private fun makeLinearInterpolator(endHeading: Double): LinearInterpolator {
        val startHeading = currentPose?.heading ?: throw PathContinuityViolationException()

        return LinearInterpolator(startHeading, Angle.normDelta(endHeading - startHeading))
    }

    private fun makeSplineInterpolator(endHeading: Double): SplineInterpolator {
        return if (currentPose == null) {
            SplineInterpolator(
                path!![s!!].heading,
                endHeading,
                path.deriv(s).heading,
                path.secondDeriv(s).heading,
                null,
                null
            )
        } else {
            SplineInterpolator(currentPose?.heading!!, endHeading)
        }
    }

    private fun addSegment(segment: PathSegment): PathBuilder {
        if (segments.isNotEmpty()) {
            val lastSegment = segments.last()
            if (!(lastSegment.end() epsilonEqualsHeading segment.start() &&
                lastSegment.endDeriv() epsilonEquals segment.startDeriv() &&
                lastSegment.endSecondDeriv().vec() epsilonEquals segment.startSecondDeriv().vec())) {
                throw PathContinuityViolationException()
            }
        } else if (currentPose == null) {
            if (!(path!![s!!] epsilonEqualsHeading segment.start() &&
                path.deriv(s) epsilonEquals segment.startDeriv() &&
                path.secondDeriv(s).vec() epsilonEquals segment.startSecondDeriv().vec())) {
                throw PathContinuityViolationException()
            }
        }

        currentPose = segment.end()
        currentTangent = segment.endTangentAngle()

        segments.add(segment)

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineTo(endPosition: Vector2d): PathBuilder {
        val line = makeLine(endPosition)
        val interpolator = makeTangentInterpolator(line)

        addSegment(PathSegment(line, interpolator))

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineToConstantHeading(endPosition: Vector2d) =
        addSegment(PathSegment(makeLine(endPosition), makeConstantInterpolator()))

    /**
     * Adds a strafe segment (i.e., a line segment with constant heading interpolation).
     *
     * @param endPosition end position
     */
    fun strafeTo(endPosition: Vector2d) = lineToConstantHeading(endPosition)

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToLinearHeading(endPose: Pose2d) =
        addSegment(PathSegment(makeLine(endPose.vec()), makeLinearInterpolator(endPose.heading)))

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToSplineHeading(endPose: Pose2d) =
        addSegment(PathSegment(makeLine(endPose.vec()), makeSplineInterpolator(endPose.heading)))

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return lineTo(start.vec() + Vector2d.polar(distance, start.heading))
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double): PathBuilder {
        forward(-distance)
        return this
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double): PathBuilder {
        val start = if (currentPose == null) {
            path!![s!!]
        } else {
            currentPose!!
        }

        return strafeTo(start.vec() + Vector2d.polar(distance, start.heading + PI / 2))
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double): PathBuilder {
        return strafeLeft(-distance)
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineTo(endPosition: Vector2d, endTangent: Double): PathBuilder {
        val spline = makeSpline(endPosition, endTangent)
        val interpolator = makeTangentInterpolator(spline)

        return addSegment(PathSegment(spline, interpolator))
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineToConstantHeading(endPosition: Vector2d, endTangent: Double) =
        addSegment(PathSegment(makeSpline(endPosition, endTangent), makeConstantInterpolator()))

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToLinearHeading(endPose: Pose2d, endTangent: Double) =
        addSegment(PathSegment(makeSpline(endPose.vec(), endTangent), makeLinearInterpolator(endPose.heading)))

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToSplineHeading(endPose: Pose2d, endTangent: Double) =
        addSegment(PathSegment(makeSpline(endPose.vec(), endTangent), makeSplineInterpolator(endPose.heading)))

    /**
     * Constructs the [Path] instance.
     */
    fun build(): Path {
        return Path(segments)
    }
}
