package com.millburnx.purePursuit.Utils

import kotlin.math.abs
import kotlin.math.sqrt

// ax + by + c = 0
open class Line(val a: Double, val b: Double, val c: Double) {
    companion object {
        fun getA(p1: Point, p2: Point): Double {
            return p1.y - p2.y
        }

        fun getB(p1: Point, p2: Point): Double {
            return p2.x - p1.x
        }

        fun getC(p1: Point, p2: Point): Double {
            val a = getA(p1, p2)
            val b = getB(p1, p2)
            return -a * p1.x - b * p1.y
        }
    }

    constructor(p1: Point, p2: Point) : this(getA(p1, p2), getB(p1, p2), getC(p1, p2))

    open fun intersections(circle: Circle): List<Intersection<Line>> {
        // translate to center the circle at the origin
        val line = translate(circle.center * -1.0)
        val a = line.a
        val b = line.b
        val c = line.c

        val r = circle.radius

        // find the closest point on the line to the circle's center
        val d0 = abs(c) / sqrt(a * a + b * b)
        if (d0 > r) return emptyList() // the point is too far away

        val x0 = -a * c / (a * a + b * b)
        val y0 = -b * c / (a * a + b * b)
        if (d0 == r) { // the point is exactly on the circle
            val p0 = Intersection(Point(x0, y0) + circle.center, this)
            return listOf(p0)
        }

        val d1 = sqrt(r * r - c * c / (a * a + b * b)) // distance from (x0, y0) to the intersection points
        val m = sqrt(d1 * d1 / (a * a + b * b))
        val x1 = x0 + b * m
        val y1 = y0 - a * m
        val p1 = Point(x1, y1) + circle.center
        val i1 = Intersection(p1, this)

        val x2 = x0 - b * m
        val y2 = y0 + a * m
        val p2 = Point(x2, y2) + circle.center
        val i2 = Intersection(p2, this)

        return listOf(i1, i2)
    }

    open fun translate(p: Point): Line {
        val a = this.a
        val b = this.b
        val c = a * p.x + b * p.y + this.c
        return Line(a, b, c)
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Line) return false
        return a == other.a && b == other.b && c == other.c
    }

    // compare x and y intercepts
    fun roughEquals(other: Line): Boolean {
        val selfX = -c / a
        val selfY = -c / b

        val otherX = -other.c / other.a
        val otherY = -other.c / other.b

        return selfX == otherX && selfY == otherY
    }

    override fun toString(): String {
        return "$a x + $b y + $c = 0"
    }

    override fun hashCode(): Int {
        var result = a.hashCode()
        result = 31 * result + b.hashCode()
        result = 31 * result + c.hashCode()
        return result
    }
}