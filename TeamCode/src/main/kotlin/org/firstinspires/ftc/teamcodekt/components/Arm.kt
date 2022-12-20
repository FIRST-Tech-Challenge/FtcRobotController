package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
object ArmConfig {
    const val FORWARDS = 135.0
    const val BACKWARDS = 39.9
    const val RESTING = 97.0
}

/**
 * This class represents an arm that can be moved forward, backward, or to a resting position.
 *
 * @param hwMap a [HardwareMap] object that contains information about the robot's hardware
 * @author KG
 */
class Arm(hwMap: HardwareMap) {
    /**
     * The servo that controls the arm. This is actually two servos acting like they are one.
     */
    private val armServo = SimpleServo(hwMap, DeviceNames.ARM_SERVO, 0.0, 180.0)

    private var targetAngle = ArmConfig.RESTING;

    /**
     * Move the arm to the forward position. Intended for intaking or reverse depositing.
     */
    fun setToForwardsPos() {
        targetAngle = ArmConfig.FORWARDS
    }

    /**
     * Move the arm to the backward position. Intended for normal depositing.
     */
    fun setToBackwardsPos() {
        targetAngle = ArmConfig.BACKWARDS
    }

    /**
     * Move the arm to the resting position. Intended for quick access to the intaking
     * and depositing positions.
     */
    fun setToRestingPos() {
        targetAngle = ArmConfig.RESTING
    }

    fun update() {
        armServo.turnToAngle(targetAngle)
    }
}
