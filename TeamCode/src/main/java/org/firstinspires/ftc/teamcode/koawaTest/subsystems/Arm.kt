package org.firstinspires.ftc.teamcode.koawaTest.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Arm(private val arm : KMotor) : Subsystem() {

    fun setPower(power: Double) {
        arm.power = power
    }

    class ArmMove(private val power: Double, private val arm : Arm) : InstantCmd({ arm.setPower(power) }, arm )
}