package com.millburnx.purePursuit.Utils

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class Utils {
    companion object {
        fun normalizeAngle(angle: Double): Double {
            return atan2(sin(angle), cos(angle))
        }

        fun pathToBeziers(path: List<Point>): MutableList<Bezier> {
            val beziers = mutableListOf<Bezier>()
            for (i in 0 until path.size - 3 step 3) {
                beziers.add(Bezier(path[i], path[i + 1], path[i + 2], path[i + 3]))
            }
            return beziers
        }
    }
}