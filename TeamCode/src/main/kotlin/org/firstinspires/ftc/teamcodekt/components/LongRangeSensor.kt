package org.firstinspires.ftc.teamcodekt.components

import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class LongRangeSensor(hwMap: HardwareMap, device: String) {
    private val sensor = hwMap.get(MB1242Ex::class.java, device)
    val distance: Double get() = sensor.getDistance(DistanceUnit.MM)/10.0
}
