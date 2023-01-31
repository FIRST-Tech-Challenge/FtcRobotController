@file:Config

package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@JvmField var WRIST_FORWARDS  = 0.81
@JvmField var WRIST_BACKWARDS = 0.151
@JvmField var WRIST_REST      = 0.4805

class Wrist {
    private val wristServo = SimpleServo(hwMap, DeviceNames.WRIST_SERVO, 0.0, 180.0)

    var wristPosition = 0.0

    fun setToBackwardsPos() {
        wristPosition = WRIST_BACKWARDS
    }

    fun setToForwardsPos() {
        wristPosition = WRIST_FORWARDS
    }

    fun setToRestingPos() {
        wristPosition = WRIST_REST
    }

    fun update() {
        wristServo.position = wristPosition
    }
}
