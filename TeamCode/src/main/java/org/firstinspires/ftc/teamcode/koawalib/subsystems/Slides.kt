package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.subsystem.Subsystem

class Slides(val motor: KMotor): Subsystem() {
    fun setPower(power: Double) {
        motor.power = power
    }
}

class SlideMove(private val power: Double, private val slides: Slides) : InstantCmd({ slides.setPower(power) }, slides )