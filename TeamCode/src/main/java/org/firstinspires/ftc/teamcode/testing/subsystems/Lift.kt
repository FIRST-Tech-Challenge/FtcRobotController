package org.firstinspires.ftc.teamcode.testing.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Lift(private val leadMotor: KMotor,
           private val secondMotor: KMotor) : Subsystem() {

    fun setPower(power: Double) {
        leadMotor.power = power
        secondMotor.power = power
    }

    class LiftMove(private val power: Double, private val lift : Lift) : InstantCmd({ lift.setPower(power) }, lift )
}