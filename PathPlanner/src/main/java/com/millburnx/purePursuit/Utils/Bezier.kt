package com.millburnx.purePursuit.Utils

import com.millburnx.purePursuit.ftcDashboard.ICanvas
import kotlin.math.max
import kotlin.math.min

class Bezier(val p1: Point, val p2: Point, val p3: Point, val p4: Point) {
    fun at(t: Double): Point {
        // get point on cubic bezier curve at t via De Casteljau's algorithm
        val q1 = p1.lerp(p2, t)
        val q2 = p2.lerp(p3, t)
        val q3 = p3.lerp(p4, t)

        val r1 = q1.lerp(q2, t)
        val r2 = q2.lerp(q3, t)

        return r1.lerp(r2, t)
    }

    fun intersections(circle: Circle, steps: Int = 10): List<Intersection<Bezier>> {
        // bounding check
        val min = Point(
            min(min(p1.x, p2.x), min(p3.x, p4.x)),
            min(min(p1.y, p2.y), min(p3.y, p4.y))
        )
        val max = Point(
            max(max(p1.x, p2.x), max(p3.x, p4.x)),
            max(max(p1.y, p2.y), max(p3.y, p4.y))
        )
        val minCircle = Point(
            circle.center.x - circle.radius,
            circle.center.y - circle.radius
        )
        val maxCircle = Point(
            circle.center.x + circle.radius,
            circle.center.y + circle.radius
        )
        if (min.x > maxCircle.x || max.x < minCircle.x || min.y > maxCircle.y || max.y < minCircle.y) {
            return emptyList()
        }
        // split bezier into segments
        val samplePoints: List<Point> = (0..steps).map { at(it.toDouble() / steps) }
        val segments: List<LineSegment> = samplePoints.zipWithNext().map { LineSegment(it.first, it.second) }
        return segments.flatMap { it.intersections(circle) }.map { Intersection(it.point, this) }
    }

    fun draw(canvas: ICanvas, steps: Int = 10) {
        var lastPoint = p1
        for (i in 1..steps) {
            val t = i.toDouble() / steps
            val point = at(t)
            canvas.strokeLine(lastPoint.x, lastPoint.y, point.x, point.y)
            lastPoint = point
        }
    }
}