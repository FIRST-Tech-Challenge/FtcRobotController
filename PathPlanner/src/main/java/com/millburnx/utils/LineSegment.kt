package com.millburnx.utils

import kotlin.math.max
import kotlin.math.min

class LineSegment(val p1: Vec2d, val p2: Vec2d) : Line(p1, p2) {
    override fun intersections(circle: Circle): List<Intersection<Line>> {
        val intersections = super.intersections(circle)
        return intersections.filter { boundingContains(it.point) }
    }

    /**
     * Returns the length of the line segment
     */
    private fun length(): Double {
        return p1.distanceTo(p2)
    }

    override fun translate(p: Vec2d): LineSegment {
        return LineSegment(p1 + p, p2 + p)
    }

    /**
     * Checks if a point is on the line segment
     */
    fun contains(p: Vec2d): Boolean {
        val d1 = p1.distanceTo(p)
        val d2 = p2.distanceTo(p)
        return d1 + d2 == length()
    }

    /**
     * Checks if a point is inside the bounding box of the line segment
     */
    private fun boundingContains(p: Vec2d): Boolean {
        val xMin = min(p1.x, p2.x)
        val xMax = max(p1.x, p2.x)
        val yMin = min(p1.y, p2.y)
        val yMax = max(p1.y, p2.y)

        return p.x in xMin..xMax && p.y in yMin..yMax
    }

    override fun equals(other: Any?): Boolean {
        if (other !is LineSegment) return false
        return p1 == other.p1 && p2 == other.p2
    }

    override fun toString(): String {
        return "($p1, $p2)"
    }

    override fun hashCode(): Int {
        var result = p1.hashCode()
        result = 31 * result + p2.hashCode()
        return result
    }
}