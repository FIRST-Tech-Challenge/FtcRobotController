package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcodekt.util.invoke

object ShortRangeSensorConfig {
    // TODO: Empirically determine model values. Requires a tuning process.
}

class ShortRangeSensor @JvmOverloads constructor(hwMap: HardwareMap, private val telemetry: Telemetry? = null) {
    private val sensor = hwMap<AnalogInput>(DeviceNames.SHORT_RANGE_SENSOR)

    val distance: Double
        get() = model(sensor.voltage).also {
            telemetry?.addData("Sensor reading", distance)
        }

    private fun model(input: Double): Double {
        return 239.664 * input - 37.1034
    }
}
