package com.millburnx.purePursuit.Utils

class LineSegment(val p1: Point, val p2: Point) : Line(p1, p2) {
    override fun intersections(circle: Circle): List<Intersection> {
        val intersections = super.intersections(circle)
        return intersections.filter { boundingContains(it.point) }
    }

    fun length(): Double {
        return p1.distanceTo(p2)
    }

    override fun translate(p: Point): LineSegment {
        return LineSegment(p1 + p, p2 + p)
    }

    fun contains(p: Point): Boolean {
        val d1 = p1.distanceTo(p)
        val d2 = p2.distanceTo(p)
        return d1 + d2 == length()
    }

    fun boundingContains(p: Point): Boolean {
        val xMin = p1.x.coerceAtMost(p2.x)
        val xMax = p1.x.coerceAtLeast(p2.x)
        val yMin = p1.y.coerceAtMost(p2.y)
        val yMax = p1.y.coerceAtLeast(p2.y)

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