package com.millburnx.utils

import com.acmerobotics.dashboard.canvas.Canvas
import java.awt.Color
import java.awt.Graphics2D
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

data class Bezier(val p0: Vec2d, val p1: Vec2d, val p2: Vec2d, val p3: Vec2d) {
    companion object {
        fun fromLine(p0: Vec2d, p1: Vec2d): Bezier {
            return Bezier(p0, p0.lerp(p1, 1.0 / 3), p1.lerp(p0, 1.0 / 3), p1)
        }

        /**
         * Generates a cubic bezier from a catmull-rom spline,
         * @param p0 the point before the start of the curve
         * @param p1 the start of the curve
         * @param p2 the end of the curve
         * @param p3 the point after the end of the curve
         * @param alpha knot parameter, 0.0 for a normal uniform cubic bezier,
         * @see <a href="https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline">Centripetal Catmull-Rom spline - Wikipedia</a>
         */
        fun fromCatmullRom(p0: Vec2d, p1: Vec2d, p2: Vec2d, p3: Vec2d, alpha: Double = 0.0): Bezier {
            val t0 = 0
            val t1 = p0.distanceTo(p1).pow(alpha) + t0
            val t2 = p1.distanceTo(p2).pow(alpha) + t1
            val t3 = p2.distanceTo(p3).pow(alpha) + t2

            val c1 = (t2 - t1) / (t2 - t0)
            val c2 = (t1 - t0) / (t2 - t0)
            val d1 = (t3 - t2) / (t3 - t1)
            val d2 = (t2 - t1) / (t3 - t1)

            val v1 = ((p1 - p0) * c1 / (t1 - t0) + (p2 - p1) * c2 / (t2 - t1)) * (t2 - t1) // velocity/derivative at p1
            val v2 = ((p2 - p1) * d1 / (t2 - t1) + (p3 - p2) * d2 / (t3 - t2)) * (t2 - t1) // velocity/derivative at p2

            return Bezier(p1, p1 + v1 / 3, p2 - v2 / 3, p2)
        }
    }

    /**
     * Get the point on the Bézier curve at t (0.0 to 1.0) using De Casteljau's algorithm
     * @see <a href="https://youtu.be/jvPPXbo87ds?t=234">De Casteljau's algorithm - Freya Holmér "The Continuity of Splines" @3:54 - YouTube</a>
     */
    fun at(t: Double): Vec2d {
        val s0p0 = p0.lerp(p1, t)
        val s0p1 = p1.lerp(p2, t)
        val s0p2 = p2.lerp(p3, t)

        val s1p0 = s0p0.lerp(s0p1, t)
        val s1p1 = s0p1.lerp(s0p2, t)

        return s1p0.lerp(s1p1, t)
    }

    /**
     * Get a list of intersections between the Bézier curve and a circle
     * @param samples the number of segments to split the Bézier curve into; curve resolution; higher is more accurate but slower
     */
    fun intersections(circle: Circle, samples: Int = 100): List<Intersection<Bezier>> {
        // bounding check
        val min = Vec2d(
            min(min(p0.x, p1.x), min(p2.x, p3.x)),
            min(min(p0.y, p1.y), min(p2.y, p3.y))
        )
        val max = Vec2d(
            max(max(p0.x, p1.x), max(p2.x, p3.x)),
            max(max(p0.y, p1.y), max(p2.y, p3.y))
        )
        val minCircle = Vec2d(
            circle.center.x - circle.radius,
            circle.center.y - circle.radius
        )
        val maxCircle = Vec2d(
            circle.center.x + circle.radius,
            circle.center.y + circle.radius
        )
        if (min.x > maxCircle.x || max.x < minCircle.x || min.y > maxCircle.y || max.y < minCircle.y) {
            return emptyList()
        }
        // split bezier into segments
        val samplePoints: List<Vec2d> = (0..samples).map { at(it.toDouble() / samples) }
        val segments: List<LineSegment> = samplePoints.zipWithNext().map { LineSegment(it.first, it.second) }
        return segments.flatMap { it.intersections(circle) }.map { Intersection(it.point, this) }
    }

    /**
     * Draw the Bézier curve on an FTC Dashboard canvas
     */
    fun draw(canvas: Canvas, samples: Int = 100) {
        var lastPoint = p0
        for (i in 1..samples) {
            val t = i.toDouble() / samples
            val point = at(t)
            canvas.strokeLine(lastPoint.x, lastPoint.y, point.x, point.y)
            lastPoint = point
        }
    }

    fun g2dDraw(g2d: Graphics2D, ppi: Double, scale: Double, color: Color) {
        g2d.color = color
        val samples = 100
        for (i in 1..samples) {
            val t = i.toDouble() / samples
            val point = at(t)
            val lastPoint = at((i - 1).toDouble() / samples)
            g2d.drawLine(
                (lastPoint.x * ppi).toInt(),
                (lastPoint.y * ppi).toInt(),
                (point.x * ppi).toInt(),
                (point.y * ppi).toInt()
            )
        }
    }
}