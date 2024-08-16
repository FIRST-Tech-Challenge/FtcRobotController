package com.millburnx.purePursuit
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Point(val x: Double, val y: Double) {
    operator fun plus(point: Point): Point {
        return Point(x + point.x, y + point.y)
    }

    operator fun minus(point: Point): Point {
        return Point(x - point.x, y - point.y)
    }

    operator fun times(factor: Double): Point {
        return Point(x * factor, y * factor)
    }

    operator fun times(point: Point): Double {
        return x * point.x + y * point.y
    }

    operator fun div(factor: Double): Point {
        return Point(x / factor, y / factor)
    }

    operator fun div(point: Point): Point {
        return Point(x / point.x, y / point.y)
    }

    fun distanceTo(other: Point): Double {
        val xDiff = x - other.x
        val yDiff = y - other.y
        return sqrt(xDiff * xDiff + yDiff * yDiff)
    }

    fun angleTo(other: Point): Double {
        return atan2(other.y - y, other.x - x)
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Point) return false
        return x == other.x && y == other.y
    }

    override fun toString(): String {
        return "($x, $y)"
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    fun rotate(heading: Double): Point {
        // Rotate the point around the origin
        val x = this.x * cos(heading) - this.y * sin(heading)
        val y = this.x * sin(heading) + this.y * cos(heading)
        return Point(x, y)
    }
}