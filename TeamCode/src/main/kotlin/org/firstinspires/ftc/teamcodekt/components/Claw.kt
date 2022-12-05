package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
object ClawConfig {
    const val INTAKE_WIDE   = 0.55
    const val INTAKE_NARROW = 0.48
    const val DEPOSIT       = 0.55
    const val CLOSE         = 0.35
}

class Claw(hwMap: HardwareMap) {
    private val clawServo = SimpleServo(hwMap, "CL", 0.0, 180.0)

    fun openForIntakeNarrow() {
        clawServo.position = ClawConfig.INTAKE_NARROW
    }

    fun openForIntakeWide() {
        clawServo.position = ClawConfig.INTAKE_WIDE
    }

    fun openForDeposit() {
        clawServo.position = ClawConfig.DEPOSIT
    }

    fun close() {
        clawServo.position = ClawConfig.CLOSE
    }
}
