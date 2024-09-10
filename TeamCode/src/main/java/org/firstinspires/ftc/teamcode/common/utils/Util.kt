package org.firstinspires.ftc.teamcode.common.utils

import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d

class Util {
    companion object {
        fun getAngleDiff(a: Pair<Vec2d, Double>, b: Vec2d): Double {
            val (aPoint, aAngle) = a
            val diff = Utils.normalizeAngle(aPoint.angleTo(b))
            return Utils.normalizeAngle(diff - aAngle)
        }
    }
}