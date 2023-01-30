package org.firstinspires.ftc.teamcodekt.components.basicranging

import com.qualcomm.robotcore.hardware.AnalogInput
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.invoke
import kotlin.math.sqrt

class ShortRangeSensor(name: String) {
    private val sensor = hwMap<AnalogInput>(name)

    val distance: Double
        get() = sqrtModel(sensor.voltage)

    val rawVoltage: Double
        get() = sensor.voltage

    private fun sqrtModel(voltage: Double): Double {
        return -219.67 * sqrt(-0.999991 * (voltage - 0.425162)) + 116.096
    }

    fun avgOf(num: Int): Double {
        return DoubleArray(num) { distance }.average()
    }
}
