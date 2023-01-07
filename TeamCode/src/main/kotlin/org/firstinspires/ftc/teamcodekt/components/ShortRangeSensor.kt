package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.invoke
import org.firstinspires.ftc.teamcodekt.components.ShortRangeSensorConfig.OFFSET
import org.firstinspires.ftc.teamcodekt.components.ShortRangeSensorConfig.SCALE

@Config
object ShortRangeSensorConfig {
//    @JvmField var OFFSET = 37.1037
//    @JvmField var SCALE = 239.664

    @JvmField var OFFSET = 35.7037
    @JvmField var SCALE = 240.264
}

class ShortRangeSensor(name: String) {
    private val sensor = hwMap<AnalogInput>(name)

    val distance: Double
        get() = model(sensor.voltage).also {
            readings[index] = it
            index = (index + 1) % readings.size
        }

    private fun model(input: Double): Double {
        return SCALE * input - OFFSET
    }

    private val readings = DoubleArray(40)
    private var index = 0

    val average: Double
        get() = readings.average()

    fun restartAvg() {
        index = 0
        readings.fill(distance)
    }
}
