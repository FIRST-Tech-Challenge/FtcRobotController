package org.firstinspires.ftc.teamcode.common.utils

import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d

class Util {
    companion object {
        fun getAngleDiff(a: Vec2d, angle: Double, b: Vec2d): Double {
            val angle = Utils.normalizeAngle(a.angleTo(b))
            return Utils.normalizeAngle(angle - angle)
        }
    }
}