package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Lift( private val leadMotor: KMotor,
            private val secondMotor: KMotor
            ) : Subsystem() {

    fun setPos(pos: Double) {
        leadMotor.setPositionTarget(pos)
    }

    override fun periodic() {
        secondMotor.power = leadMotor.power
    }
}