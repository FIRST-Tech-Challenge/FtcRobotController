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
    private val bezier: List<Bezier> = Utils.pathToBeziers(path);
    private val lastPoint: Vec2d = path.last();
    private var lastSegment: Int = 0;
    private var lastIntersection: Intersection<Bezier> = Intersection(path[0], bezier[0]);
    var isFinished = false

    fun calc(pos: Vec2d, heading: Double): Vec2d {
        val distanceToLast = pos.distanceTo(lastPoint);
        if (distanceToLast < threshold) {
            isFinished = true;
            return lastIntersection.point
        }
        if (distanceToLast <= lookahead) {
            return lastPoint
        }
        val lookaheadCircle = Circle(pos, lookahead);
        val remainingPath = bezier.subList(lastSegment, bezier.size);
        val intersections = remainingPath.flatMap { it.intersections(lookaheadCircle) }
        val closestIntersection = intersections.minByOrNull { abs(getAngleDiff(pos, heading, it.point)) }
        val targetIntersection = closestIntersection ?: lastIntersection
        lastSegment = bezier.indexOf(targetIntersection.line) // pruning
        lastIntersection = targetIntersection

        return lastIntersection.point
    }

    companion object {
        fun getAngleDiff(a: Vec2d, angle: Double, b: Vec2d): Double {
            val angle = Utils.normalizeAngle(a.angleTo(b))
            return Utils.normalizeAngle(angle - angle)
        }
    }
}