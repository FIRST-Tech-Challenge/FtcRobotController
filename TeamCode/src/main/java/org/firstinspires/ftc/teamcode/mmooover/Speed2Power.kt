package org.firstinspires.ftc.teamcode.mmooover

import kotlin.math.abs

class Speed2Power(val thresh: Double) {
    companion object {
        const val EPSILON = 0.001
    }

    constructor(thresh: Number) : this(thresh.toDouble())

    fun speed2power(speed: Double) = when {
        abs(speed) < EPSILON -> 0.0
        speed > 0 -> thresh + ((1 - thresh) * speed)
        speed < 0 -> -thresh + ((1 - thresh) * speed)
        else -> throw IllegalArgumentException()
    }
}