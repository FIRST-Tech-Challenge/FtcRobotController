@file:Config

package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@JvmField var CLAW_INTAKE_WIDE   = 0.6475
@JvmField var CLAW_INTAKE_NARROW = 0.568
@JvmField var CLAW_DEPOSIT       = 0.666
@JvmField var CLAW_CLOSE         = 0.425

class Claw {
    private val clawServo = SimpleServo(hwMap, DeviceNames.CLAW_SERVO, 0.0, 180.0)

    private var targetPos = CLAW_CLOSE

    fun openForIntakeNarrow() {
        targetPos = CLAW_INTAKE_NARROW
    }

    fun openForIntakeWide() {
        targetPos = CLAW_INTAKE_WIDE
    }

    fun openForIntakeKindaWide() {
        targetPos = 0.62
    }

    fun openForDeposit() {
        targetPos = CLAW_DEPOSIT
    }

    fun close() {
        targetPos = CLAW_CLOSE
    }

    fun update() {
        clawServo.position = targetPos
    }
}
