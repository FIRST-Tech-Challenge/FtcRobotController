package org.firstinspires.ftc.teamcode.baseClasses

import kotlin.math.PI

class UnitAngle private constructor(deg: Double) {
    private val deg = deg

    companion object {
        @JvmStatic
        fun degrees(value: Double): UnitAngle = UnitAngle(value)

        @JvmStatic
        fun radians(value: Double): UnitAngle = UnitAngle(Math.toRadians(value))
    }

    val degrees: Double get() = (deg + 360) % 360
    val radians: Double get() = Math.toRadians(deg)

    val quadrant: Quadrant
        get() {
            val normalized = ((degrees % 360) + 360) % 360
            return when {
                normalized <= 90 -> Quadrant.I
                normalized <= 180 -> Quadrant.IV
                normalized <= 270 -> Quadrant.III
                normalized <= 360 -> Quadrant.II
                else -> Quadrant.I
            }
        }


    enum class Quadrant { I, II, III, IV }
}