package com.millburnx.purePursuit.Utils

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
}