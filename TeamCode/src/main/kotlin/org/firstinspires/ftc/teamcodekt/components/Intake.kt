package org.firstinspires.ftc.teamcodekt.components

import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

//@Config
object IntakeConfig {
    const val INTAKING = -1.0
    const val REVERSED =  1.0
    const val DISABLED =  0.0
}

class Intake(hwMap: HardwareMap?) {
    private val intakeServo = CRServo(hwMap, "IN")

    fun enable() {
        intakeServo.set(IntakeConfig.INTAKING)
    }

    fun disable() {
        intakeServo.set(IntakeConfig.DISABLED)
    }

    fun reverse() {
        intakeServo.set(IntakeConfig.REVERSED)
    }
}
