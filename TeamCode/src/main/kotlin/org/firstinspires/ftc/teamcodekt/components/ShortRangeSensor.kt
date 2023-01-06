package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import ftc.rogue.blacksmith.util.kt.invoke

object ShortRangeSensorConfig {
    // TODO: Empirically determine model values. Requires a tuning process.
}

class ShortRangeSensor(hwMap: HardwareMap) {
    private val sensor = hwMap<AnalogInput>(DeviceNames.SHORT_RANGE_SENSOR)

    val distance: Double get() = model(sensor.voltage)

    private fun model(input: Double): Double {
        return 239.664 * input - 37.1034
    }
}
