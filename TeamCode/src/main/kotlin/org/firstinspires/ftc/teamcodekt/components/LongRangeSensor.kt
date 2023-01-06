package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.robotcore.hardware.HardwareMap
import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class LongRangeSensor(hwMap: HardwareMap, device: String) {
    private val sensor = MB1242Ex(hwMap.i2cDeviceSynch.get(device))
    val distance: Double get() = sensor.getDistance(DistanceUnit.CM)
}
