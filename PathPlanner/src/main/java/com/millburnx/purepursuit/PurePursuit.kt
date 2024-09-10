package com.millburnx.purepursuit

import com.millburnx.utils.Bezier
import com.millburnx.utils.Circle
import com.millburnx.utils.Intersection
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import kotlin.math.abs

class PurePursuit(
    path: List<Vec2d>,
    val lookahead: Double,
    val threshold: Double = 1.0,
) {
    private val bezier: List<Bezier> = Utils.pathToBeziers(path)
    private val lastPoint: Vec2d = path.last()
    private var lastSegment: Int = 0
    private var lastIntersection: Intersection<Bezier> = Intersection(path[0], bezier[0])
    var isFinished = false

    fun getIntersections(lookahead: Circle, segments: List<Bezier>): List<Intersection<Bezier>> {
        return segments.flatMap { it.intersections(lookahead) }
    }

    fun getTarget(
        intersections: List<Intersection<Bezier>>,
        pos: Vec2d,
        heading: Double
    ): Intersection<Bezier> {
        val closest = intersections.minByOrNull {
            abs(getAngleDiff((pos to heading), it.point))
        }
        return closest ?: lastIntersection
    }

    fun updateLastIntersection(lastIntersection: Intersection<Bezier>) {
        this.lastIntersection = lastIntersection
        this.lastSegment = bezier.indexOf(lastIntersection.line)
    }

    fun calc(pos: Vec2d, heading: Double): PurePursuitData {
        val distanceToLast = pos.distanceTo(lastPoint)
        if (distanceToLast < threshold) {
            isFinished = true

            @Suppress("KotlinConstantConditions")
            return PurePursuitData(
                lastIntersection.point,
                isFinished,
                bezier,
                listOf(),
                listOf(lastIntersection)
            )
        }
        if (distanceToLast <= lookahead) {
            return PurePursuitData(
                lastPoint,
                isFinished,
                bezier,
                listOf(Bezier.fromLine(pos, lastPoint)),
                listOf(lastIntersection)
            )
        }
        val lookaheadCircle = Circle(pos, lookahead)
        val remainingPath = bezier.subList(lastSegment, bezier.size)
        val intersections = getIntersections(lookaheadCircle, remainingPath)
        val targetIntersection = getTarget(intersections, pos, heading)
        updateLastIntersection(targetIntersection)

        return PurePursuitData(
            targetIntersection.point,
            isFinished,
            bezier,
            remainingPath,
            intersections
        )
    }

    companion object {
        fun getAngleDiff(a: Pair<Vec2d, Double>, b: Vec2d): Double {
            val (aPoint, aAngle) = a
            val diff = Utils.normalizeAngle(aPoint.angleTo(b))
            return Utils.normalizeAngle(diff - aAngle)
        }
    }
}

data class PurePursuitData(
    val target: Vec2d,
    val isFinished: Boolean,
    val path: List<Bezier>,
    val remainingPath: List<Bezier>,
    val intersections: List<Intersection<Bezier>>,
)