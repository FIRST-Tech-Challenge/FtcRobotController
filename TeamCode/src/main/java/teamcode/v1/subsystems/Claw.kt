package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.subsystem.Subsystem

class Claw(private val servo: KServo,
//           private val distanceSensor: KDistanceSensor
) : Subsystem() {
//    val readyToGrab get() = distanceSensor.lastRead < ClawConstants.distanceThreshold

    fun setPos(pos: Double) {
        servo.position = pos
    }
}