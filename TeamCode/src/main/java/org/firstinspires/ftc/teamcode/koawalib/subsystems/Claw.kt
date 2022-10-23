package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.hardware.sensor.KDistanceSensor
import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.subsystem.Subsystem

class Claw(private val servo: KServo,
           private val distanceSensor: KDistanceSensor
) : Subsystem() {
    companion object {
        val openPos : Double = TODO()
        val closePos : Double = TODO()
        val distanceThreshold: Double = TODO()
    }

    val readyToGrab get() = distanceSensor.lastRead < distanceThreshold

    fun setPos(pos: Double) {
        servo.position = pos
    }
}