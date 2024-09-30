package org.firstinspires.ftc.teamcode.snav.pathing

import java.util.Objects

open class Point(val x: Int, val y: Int) {
    // Only check x and y to determine if a point is equal (PathPoints and Points with same coordinate are equal)
    override fun equals(other: Any?): Boolean {
        return if (other is Point) {
            x == other.x && y == other.y
        } else {
            false
        }
    }

    override fun hashCode(): Int {
        return Objects.hash(x, y)
    }
}

class PathPoint(x: Int, y: Int, val previous: PathPoint? = null, val field: Field) : Point(x, y) {

    fun generateNeighbors(): MutableList<PathPoint> {
        var neighbors: MutableList<PathPoint> = mutableListOf()
        // If the point is traversable add it to the neighbors list
        if (field.isTraversable(Point(x, y + 1))) neighbors.add(PathPoint(x, y + 1, this, field))
        if (field.isTraversable(Point(x, y - 1))) neighbors.add(PathPoint(x, y - 1, this, field))
        if (field.isTraversable(Point(x - 1, y))) neighbors.add(PathPoint(x - 1, y, this, field))
        if (field.isTraversable(Point(x + 1, y))) neighbors.add(PathPoint(x + 1, y, this, field))

        return neighbors
    }
}