package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Lift( private val leadMotor: KMotor,
            private val secondMotor: KMotor,
            private val thirdMotor: KMotor
            ) : Subsystem() {
    companion object {
        @JvmField var ticksPerUnit = 0.0
        @JvmField var homePos = 0.0
        @JvmField var groundPos = 0.0
        @JvmField var lowPos = 0.0
        @JvmField var midPos = 0.0
        @JvmField var highPos = 0.0
        @JvmField var kP = 0.0
        @JvmField var kI = 0.0
        @JvmField var kD = 0.0
        @JvmField var kS = 0.0
        @JvmField var kV = 0.0
        @JvmField var kA = 0.0
        @JvmField var kG = 0.0
        @JvmField var maxVel = 0.0
        @JvmField var maxAccel = 0.0
        @JvmField var disabledPosition = 0.0
        @JvmField var allowedPositionError = 0.0
    }
    fun setPos(pos: Double) {
        leadMotor.setProfileTarget(pos)
    }

    override fun periodic() {
        secondMotor.power = leadMotor.power
        thirdMotor.power = leadMotor.power
    }
}