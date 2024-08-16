package com.millburnx.purePursuit

class Circle(val center: Point, val radius: Double) {

    fun contains(p: Point): Boolean {
        return center.distanceTo(p) <= radius
    }

    fun boundingContains(p: Point): Boolean {
        val min = center - Point(radius, radius)
        val max = center - Point(radius, radius)

        return p.x in min.x..max.x && p.y in min.y..max.y
    }

    override fun equals(other: Any?): Boolean {
        if (other !is Circle) return false
        return center == other.center && radius == other.radius
    }

    override fun toString(): String {
        return "($center, r=$radius)"
    }

    override fun hashCode(): Int {
        var result = center.hashCode()
        result = 31 * result + radius.hashCode()
        return result
    }
}