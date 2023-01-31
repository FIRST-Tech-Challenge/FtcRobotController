@file:Config

package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.HardwareMap

@JvmField var TARGET_VOLTAGE = 14.0
@JvmField var VS_P = 0.0
@JvmField var VS_I = 0.0
@JvmField var VS_D = 0.0
@JvmField var VS_F = 0.0

class VoltageScaler(hwMap: HardwareMap) {
    private val voltagePIDF = PIDFController(
        VS_P, VS_I,
        VS_D, VS_F,
    )

    private val voltageSensor = hwMap.voltageSensor.iterator().next()

    val voltageCorrection: Double
        get() {
            return voltagePIDF.calculate(voltageSensor.voltage, TARGET_VOLTAGE)
        }
}
