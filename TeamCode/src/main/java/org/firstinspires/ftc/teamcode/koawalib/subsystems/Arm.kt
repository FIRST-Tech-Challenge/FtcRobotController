package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.hardware.servo.KServo
import com.asiankoala.koawalib.subsystem.Subsystem

class Arm(val servo: KServo) : Subsystem() {
    companion object {
        const val restPos = 0.0
        const val outtakePos = 1.0
    }

    fun rest(){
        servo.position = restPos
    }

    fun outtake(){
        servo.position = outtakePos
    }

    class ArmReset(arm : Arm) : InstantCmd(arm::rest, arm)
    class ArmOut(arm : Arm) : InstantCmd(arm::outtake, arm)
}