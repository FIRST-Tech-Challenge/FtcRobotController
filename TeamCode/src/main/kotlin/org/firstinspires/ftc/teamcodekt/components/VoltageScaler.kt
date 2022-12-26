package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
object VoltageScalerConfig {
    const val TARGET_VOLTAGE = 14.0
    const val P = 0.0
    const val I = 0.0
    const val D = 0.0
    const val F = 0.0
}

class VoltageScaler(hwMap: HardwareMap) {
    private val voltagePIDF = PIDFController(
        VoltageScalerConfig.P, VoltageScalerConfig.I,
        VoltageScalerConfig.D, VoltageScalerConfig.F
    )

    private val voltageSensor = hwMap.voltageSensor.iterator().next()

    val voltageCorrection: Double
        get() {
            return voltagePIDF.calculate(voltageSensor.voltage, VoltageScalerConfig.TARGET_VOLTAGE)
        }
}
