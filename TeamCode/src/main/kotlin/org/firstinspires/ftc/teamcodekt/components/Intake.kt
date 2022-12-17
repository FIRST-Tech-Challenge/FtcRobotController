package org.firstinspires.ftc.teamcodekt.components

import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

//@Config
object IntakeConfig {
    const val INTAKING = -1.0
    const val REVERSED =  1.0
    const val DISABLED =  0.0
}

/**
 * This class represents an intake that can be enabled, disabled, or reversed.
 *
 * @param hwMap a [HardwareMap] object that contains information about the robot's hardware
 * @author KG
 */
class Intake(hwMap: HardwareMap?) {
    private val intakeServo = CRServo(hwMap, DeviceNames.INTAKE_SERVO)

    /**
     * Enable the intake. Used for intaking cones (great for simply driving into the cones, which
     * then subsequently get practically teleported in).
     */
    fun enable() {
        intakeServo.set(IntakeConfig.INTAKING)
    }

    /**
     * Disable the intake.
     */
    fun disable() {
        intakeServo.set(IntakeConfig.DISABLED)
    }

    /**
     * Reverse the intake. Should probably never be used, as it shoots out cones instead of just
     * dropping them.
     */
    fun reverse() {
        intakeServo.set(IntakeConfig.REVERSED)
    }
}
