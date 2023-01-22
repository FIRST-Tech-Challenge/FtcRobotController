package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@Config
object IntakeConfig {
    @JvmField var INTAKING = -1.0
    @JvmField var REVERSED =  1.0
    @JvmField var DISABLED =  0.0
}

class Intake {
    private val intakeServo = CRServo(hwMap, DeviceNames.INTAKE_SERVO)

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
