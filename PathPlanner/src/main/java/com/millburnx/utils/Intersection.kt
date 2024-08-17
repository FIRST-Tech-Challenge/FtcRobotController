package com.millburnx.utils

/**
 * Represents a point of intersection on a line
 */
class Intersection<Line>(val point: Vec2d, val line: Line) {
    override fun toString(): String {
        return "Intersection(point=$point, line=$line)"
    }
}