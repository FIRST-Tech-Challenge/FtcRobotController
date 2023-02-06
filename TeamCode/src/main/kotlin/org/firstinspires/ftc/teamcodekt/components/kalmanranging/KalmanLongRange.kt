@file:Suppress("PropertyName")

package org.firstinspires.ftc.teamcodekt.components.kalmanranging

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import ftc.rogue.blacksmith.util.kalman.KalmanFilter
import kotlin.math.sqrt

@Config
class KalmanLongRange(name: String) {
    @JvmField
    var R = 7.0

    @JvmField
    var Q = 6.0

    private val sensor = hwMap.get(MB1242Ex::class.java, name)
    private val kalman = KalmanFilter(R, Q) // TODO: Tweak these values to try and get noise down

    val distanceRaw: Double
        get() = sensor.getDistance(DistanceUnit.CM)

    fun distanceLeft(): Double {
        return kalman.filter(-108.795 * sqrt(-0.0237234 * (distanceRaw - 206.937)) + 276.65)
    }

    fun avgOfLeft(num: Int): Double {
        return DoubleArray(num) { distanceLeft() }.average()
    }
}
