@file:Config

package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.motors.CRServo
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@JvmField var INTAKE_INTAKING = -1.0
@JvmField var INTAKE_REVERSED =  1.0
@JvmField var INTAKE_DISABLED =  0.0

class Intake {
    private val intakeServo = CRServo(hwMap, DeviceNames.INTAKE_SERVO)

    fun enable() {
        intakeServo.set(INTAKE_INTAKING)
    }

    fun disable() {
        intakeServo.set(INTAKE_DISABLED)
    }

    fun reverse() {
        intakeServo.set(INTAKE_REVERSED)
    }
}
