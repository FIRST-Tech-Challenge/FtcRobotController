package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import ftc.rogue.blacksmith.util.kt.invoke


class ShortRangeSensor(hwMap: HardwareMap, device: String) {
    private val sensor = hwMap<AnalogInput>(device)

    val distance: Double get() = model(sensor.voltage)

    private fun model(input: Double): Double {
        return 239.664 * input - 37.1034
    }
}
