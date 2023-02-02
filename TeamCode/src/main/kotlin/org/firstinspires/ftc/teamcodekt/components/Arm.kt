@file:Config

package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@JvmField var ARM_FORWARDS  = 136.5
@JvmField var ARM_BACKWARDS = 40.5
@JvmField var ARM_RESTING   = 96.0

class Arm {
    private val armServo = SimpleServo(hwMap, DeviceNames.ARM_SERVO, 0.0, 180.0)

    var targetAngle = ARM_RESTING

    fun setToForwardsPos() {
        targetAngle = ARM_FORWARDS
    }

    fun setToBackwardsPos() {
        targetAngle = ARM_BACKWARDS
    }

    fun setToForwardsAngledPos() {
        targetAngle = ARM_FORWARDS - 18
    }

    fun setToBackwardsPosButLikeSliiiightlyHigher() {
        targetAngle = 42.0
    }

    fun setToRestingPos() {
        targetAngle = ARM_RESTING
    }

    fun update() {
        armServo.turnToAngle(targetAngle)
    }
}
