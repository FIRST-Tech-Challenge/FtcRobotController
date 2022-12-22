package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
object ClawConfig {
    @JvmField var INTAKE_WIDE   = 0.66
    @JvmField var INTAKE_NARROW = 0.575
    @JvmField var DEPOSIT       = 0.666
    @JvmField var CLOSE         = 0.35
}

/**
 * This class represents a claw that can be opened in different positions for different tasks.
 *
 * @param hwMap a [HardwareMap] object that contains information about the robot's hardware
 * @author KG
 */
class Claw(hwMap: HardwareMap) {
    private val clawServo = SimpleServo(hwMap, DeviceNames.CLAW_SERVO, 0.0, 180.0)

    private var targetPos = ClawConfig.CLOSE

    /**
     * Open the claw to the narrow position for intaking objects. Used primarily for tele as it
     * allows for sucking in the cone at high speeds.
     */
    fun openForIntakeNarrow() {
        targetPos = ClawConfig.INTAKE_NARROW
    }

    /**
     * Open the claw to the wide position for intaking objects. Intended mainly for auto to allow
     * for a higher chance of grabbing the cone.
     */
    fun openForIntakeWide() {
        targetPos = ClawConfig.INTAKE_WIDE
    }

    /**
     * Open the claw to the deposit position for depositing objects.
     */
    fun openForDeposit() {
        targetPos = ClawConfig.DEPOSIT
    }

    /**
     * Close the claw.
     */
    fun close() {
        targetPos = ClawConfig.CLOSE
    }

    fun update() {
        clawServo.position = targetPos
    }
}
