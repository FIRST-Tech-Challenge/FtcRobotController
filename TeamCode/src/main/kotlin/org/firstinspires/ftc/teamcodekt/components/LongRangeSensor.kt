package org.firstinspires.ftc.teamcodekt.components

import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.sqrt

class LongRangeSensor(name: String) {
    private val sensor = hwMap.get(MB1242Ex::class.java, name)

    val distanceRaw: Double
        get() = sensor.getDistance(DistanceUnit.CM)

    fun distanceLeft(): Double {
        return -108.795 * sqrt(-0.0237234 * (distanceRaw - 206.937)) + 276.65
    }

    fun avgOfLeft(num: Int): Double {
        return DoubleArray(num) { distanceLeft() }.average()
    }
}
