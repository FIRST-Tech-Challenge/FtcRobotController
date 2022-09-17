package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap

interface Subsystem {
    fun init(hardwareMap: HardwareMap)
    fun update()
    fun updateTelemetry()
    fun reset()
}