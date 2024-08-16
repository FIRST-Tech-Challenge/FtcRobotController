package com.millburnx.purePursuit.Utils

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class Utils {
    companion object {
        fun normalizeAngle(angle: Double): Double {
            return atan2(sin(angle), cos(angle))
        }
    }
}