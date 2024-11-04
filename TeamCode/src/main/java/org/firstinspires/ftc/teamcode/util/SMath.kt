package org.firstinspires.ftc.teamcode.util

fun clampf(value: Double, min: Double, max: Double): Double {
    if (value in (min + 1)..<max) {
        return value
    } else if (value < min) {
        return min
    }

    return max
}

fun clampi(value: Int, min: Int, max: Int): Int {
    if (value in (min + 1)..<max) {
        return value
    } else if (value < min) {
        return min
    }

    return max
}