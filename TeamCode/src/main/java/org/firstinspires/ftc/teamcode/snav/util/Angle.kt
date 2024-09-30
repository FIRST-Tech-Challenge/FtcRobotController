package org.firstinspires.ftc.teamcode.snav.util

import kotlin.math.abs

class Angle(angle: Double) {
    var angle: Double = angle
        set(value) {
            field = normalizeAngle(value)
        }

    // Normalize the angle [-180, -180]
    fun normalizeAngle(x: Double): Double {
        return ((x % 360 + 360) % 360) - 180
    }

    fun getDistance(other: Angle): Double {
        return abs(other.angle - angle)
    }

    fun getMinimumRotation(other: Angle): Double {
        val difference: Double = other.angle - angle
        return normalizeAngle(difference)
    }

    override fun toString(): String {
        return "Angle: $angle"
    }
}