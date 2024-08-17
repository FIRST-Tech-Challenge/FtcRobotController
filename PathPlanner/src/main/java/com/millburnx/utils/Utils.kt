package com.millburnx.utils

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class Utils {
    companion object {
        /**
         * Normalize an angle to be between -pi and pi radians (-180 to 180 degrees)
         */
        fun normalizeAngle(angle: Double): Double {
            return atan2(sin(angle), cos(angle))
        }

        /**
         * Converts a list of points to a list of Béziers
         */
        fun pathToBeziers(path: List<Vec2d>): MutableList<Bezier> {
            val beziers = mutableListOf<Bezier>()
            for (i in 0 until path.size - 3 step 3) {
                beziers.add(Bezier(path[i], path[i + 1], path[i + 2], path[i + 3]))
            }
            return beziers
        }
    }
}