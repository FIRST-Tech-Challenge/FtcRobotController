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

/**
 * This class represents a claw that can be opened in different positions for different tasks.
 *
 * @param hwMap a [HardwareMap] object that contains information about the robot's hardware
 * @author KG
 */
class Claw(hwMap: HardwareMap) {
    private val clawServo = SimpleServo(hwMap, "CL", 0.0, 180.0)

    /**
     * Open the claw to the narrow position for intaking objects. Used primarily for tele as it
     * allows for sucking in the cone at high speeds.
     */
    fun openForIntakeNarrow() {
        clawServo.position = ClawConfig.INTAKE_NARROW
    }

    /**
     * Open the claw to the wide position for intaking objects. Intended mainly for auto to allow
     * for a higher chance of grabbing the cone.
     */
    fun openForIntakeWide() {
        clawServo.position = ClawConfig.INTAKE_WIDE
    }

    /**
     * Open the claw to the deposit position for depositing objects.
     */
    fun openForDeposit() {
        clawServo.position = ClawConfig.DEPOSIT
    }

    /**
     * Close the claw.
     */
    fun close() {
        clawServo.position = ClawConfig.CLOSE
    }
}
