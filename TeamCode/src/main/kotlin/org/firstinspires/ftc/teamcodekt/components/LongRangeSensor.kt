package org.firstinspires.ftc.teamcodekt.components

import com.outoftheboxrobotics.photoncore.Neutrino.MB1242.MB1242Ex
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class LongRangeSensor(name: String) {
    private val sensor = hwMap.get(MB1242Ex::class.java, name)

    val distance: Double
        get() = sensor.getDistance(DistanceUnit.CM)
}
