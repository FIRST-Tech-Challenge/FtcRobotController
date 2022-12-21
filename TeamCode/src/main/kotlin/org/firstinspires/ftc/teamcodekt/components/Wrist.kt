package org.firstinspires.ftc.teamcodekt.components

import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConstants.WristConfig

class Wrist(hwMap: HardwareMap) {
    private val wristServo = SimpleServo(hwMap, DeviceNames.WRIST_SERVO, 0.0, 180.0)

    var wristPosition = 0.0

    fun setToBackwardsPos() {
        wristPosition = WristConfig.BACKWARDS
    }

    fun setToForwardsPos() {
        wristPosition = WristConfig.FORWARDS
    }

    fun setToRestingPos() {
        wristPosition = WristConfig.REST
    }

    fun update() {
        wristServo.position = wristPosition
    }
}
