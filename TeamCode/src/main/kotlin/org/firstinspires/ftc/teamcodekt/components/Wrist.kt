package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
object WristConfig {
    const val FORWARDS  = 0.86
    const val BACKWARDS = 0.185
    const val REST      = 0.5
}

class Wrist(hwMap: HardwareMap) {
    private val wristServo = SimpleServo(hwMap, DeviceNames.WRIST_SERVO, 0.0, 180.0)

    var wristPosition = 0.0

    fun setToBackwardsPos() {
        wristPosition = WristConfig.FORWARDS
    }

    fun setToForwardsPos() {
        wristPosition = WristConfig.BACKWARDS
    }

    fun setToRestingPos() {
        wristPosition = WristConfig.REST
    }

    fun update() {
        wristServo.position = wristPosition
    }
}
