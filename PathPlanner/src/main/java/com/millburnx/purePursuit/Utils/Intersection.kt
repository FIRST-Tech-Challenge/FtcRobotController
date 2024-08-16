package com.millburnx.purePursuit.Utils

class Intersection<Line>(val point: Point, val line: Line) {
    override fun toString(): String {
        return "Intersection(point=$point, line=$line)"
    }
}