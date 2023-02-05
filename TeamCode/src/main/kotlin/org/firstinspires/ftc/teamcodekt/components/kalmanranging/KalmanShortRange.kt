@file:Suppress("PropertyName")

package org.firstinspires.ftc.teamcodekt.components.kalmanranging

import com.qualcomm.robotcore.hardware.AnalogInput
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.invoke
import ftc.rogue.blacksmith.util.KalmanFilter
import kotlin.math.sqrt

class KalmanShortRange(name: String) {
    @JvmField
    var R = 7.0

    @JvmField
    var Q = 6.0

    // TODO: Tune these values - not sure how they'll be, might work with less

    private val sensor = hwMap<AnalogInput>(name)
    private val kalman = KalmanFilter(R, Q)

    val distance: Double
        get() = kalman.filter(sqrtModel(sensor.voltage))

    val rawVoltage: Double
        get() = sensor.voltage

    private fun sqrtModel(voltage: Double): Double {
        return -219.67 * sqrt(-0.999991 * (voltage - 0.425162)) + 116.096
    }

    fun avgOf(num: Int): Double {
        return DoubleArray(num) { distance }.average()
    }
}
