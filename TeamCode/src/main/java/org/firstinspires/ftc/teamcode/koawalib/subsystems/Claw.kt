package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.subsystem.Subsystem

class Claw(val servo: KServo) : Subsystem() {
    companion object {
        const val openPos = 1.0
        const val closePos = 0.65
    }

    fun open(){
        servo.position = openPos
    }

    fun close(){
        servo.position = closePos
    }

    class ClawOpen(claw : Claw) : InstantCmd(claw::open, claw)
    class ClawClose(claw : Claw) : InstantCmd(claw::close, claw)
}