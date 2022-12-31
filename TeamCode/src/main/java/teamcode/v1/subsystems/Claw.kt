package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.acmerobotics.dashboard.config.Config
import com.asiankoala.koawalib.hardware.sensor.KDistanceSensor
import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.subsystem.Subsystem
import org.firstinspires.ftc.teamcode.koawalib.constants.ClawConstants

class Claw(private val servo: KServo,
//           private val distanceSensor: KDistanceSensor
) : Subsystem() {
//    val readyToGrab get() = distanceSensor.lastRead < ClawConstants.distanceThreshold

    fun setPos(pos: Double) {
        servo.position = pos
    }
}