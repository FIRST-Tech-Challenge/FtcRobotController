package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@Config
object ClawConfig {
    @JvmField var INTAKE_WIDE   = 0.6475
    @JvmField var INTAKE_NARROW = 0.568
    @JvmField var DEPOSIT       = 0.666
    @JvmField var CLOSE         = 0.425
}

class Claw {
    private val clawServo = SimpleServo(hwMap, DeviceNames.CLAW_SERVO, 0.0, 180.0)

    private var targetPos = ClawConfig.CLOSE

    fun openForIntakeNarrow() {
        targetPos = ClawConfig.INTAKE_NARROW
    }

    fun openForIntakeWide() {
        targetPos = ClawConfig.INTAKE_WIDE
    }

    fun openForIntakeKindaWide() {
        targetPos = 0.63
    }

    fun openForDeposit() {
        targetPos = ClawConfig.DEPOSIT
    }

    fun close() {
        targetPos = ClawConfig.CLOSE
    }

    fun update() {
        clawServo.position = targetPos
    }
}
