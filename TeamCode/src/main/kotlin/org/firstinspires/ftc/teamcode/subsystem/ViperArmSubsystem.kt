package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.hardware.HardwareManager
import org.firstinspires.ftc.teamcode.util.MotorGroup

class ViperArmSubsystem(
    hardware: HardwareManager,
    private val telemetry: MultipleTelemetry? = null
) : SubsystemBase() {
    private val extensionMotorGroup = hardware.viperExtensionMotorGroup
    private val rotationMotorGroup = hardware.viperRotationMotorGroup

    init {
        extensionMotorGroup?.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        extensionMotorGroup?.resetEncoder()
        extensionMotorGroup?.setRunMode(Motor.RunMode.RawPower)
//        extensionMotorGroup?.positionCoefficient = EXTENSION_KP

        rotationMotorGroup?.setRunMode(Motor.RunMode.RawPower)
        rotationMotorGroup?.resetEncoder()
        rotationMotorGroup?.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
//        rotationMotorGroup?.positionCoefficient = ROTATION_KP
    }

    fun setExtensionMotorGroupPower(power: Double) {
        extensionMotorGroup?.set(getBoundedPower(power))
    }

    fun setRotationMotorGroupPower(power: Double) {
        rotationMotorGroup?.set(getBoundedPower(power))
    }

    fun getExtensionMotorGroupPosition(): Double = extensionMotorGroup?.get() ?: 0.0

    fun getRotationMotorGroupPosition(): Double = rotationMotorGroup?.get() ?: 0.0

    fun extendToPosition(position: Int) {
        val safePosition = getBoundedPosition(position, EXTENSION_MIN_POSITION, EXTENSION_MAX_POSITION)

        safelyGoToPosition(extensionMotorGroup, safePosition, EXTENSION_SPEED)
    }

    fun rotateToPosition(position: Int) {
        val safePosition = getBoundedPosition(position, ROTATION_MIN_POSITION, ROTATION_MAX_POSITION)

        safelyGoToPosition(rotationMotorGroup, safePosition, ROTATION_SPEED)
    }

    fun extendLowerBasket() = extendToPosition(EXTENSION_LOWER_BASKET_POSITION)

    fun extendFully() = extendToPosition(EXTENSION_MAX_POSITION)

    fun retract() = extendToPosition(EXTENSION_MIN_POSITION)

    fun rotateHome() = rotateToPosition(ROTATION_MIN_POSITION)

    fun rotateTop() = rotateToPosition(ROTATION_MAX_POSITION)

    fun deliverTopBasket() {
        rotateTop()
        extendFully()
    }

    fun deliverLowerBasket() {
        rotateTop()
        extendLowerBasket()
    }

    fun home() {
        retract()
        rotateHome()
    }

    private fun safelyGoToPosition(
        motorGroup: MotorGroup?,
        targetPosition: Int,
        motorSpeed: Double
    ) = motorGroup?.let {
        it.setTargetPosition(targetPosition)
        it.set(motorSpeed)
    }

    private fun getBoundedPosition(position: Int, min: Int, max: Int): Int {
        return if (position > max) {
            max
        } else if (position < min) {
            min
        } else {
            position
        }
    }

    private fun getBoundedPower(power: Double, min: Double = -1.0, max: Double = 1.0): Double {
        return if (power > max) {
            max
        } else if (power < min) {
            min
        } else {
            power
        }
    }

    companion object {
        private const val EXTENSION_SPEED = 1.0
        private const val EXTENSION_MIN_POSITION = 0
        private const val EXTENSION_LOWER_BASKET_POSITION = 5
        private const val EXTENSION_MAX_POSITION = 10

        private const val ROTATION_SPEED = 1.0
        private const val ROTATION_MIN_POSITION = 0
        private const val ROTATION_MAX_POSITION = 10

        private const val EXTENSION_KP = 0.0
        private const val ROTATION_KP = 0.0
    }
}