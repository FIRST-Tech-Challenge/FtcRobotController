package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class ActiveIntakeSubsystem(
    private val hardware: HardwareManager,
    private val telemetry: MultipleTelemetry? = null
) : SubsystemBase() {
    fun rotateLeft() {
        hardware.intakeRotateServo?.position = 0.0
    }

    fun rotateHome() {
        hardware.intakeRotateServo?.position = 0.0
    }

    fun rotateToBasket() {
        hardware.intakeRotateServo?.position = 0.66
    }

    fun rotateRight() {
        hardware.intakeRotateServo?.position = 1.0
    }

    fun rotateIncrementDown() {
        val currentPosition = hardware.intakeRotateServo?.position
        if (currentPosition != null && currentPosition < WRIST_MAX) {
            hardware.intakeRotateServo?.position = currentPosition + WRIST_STEP
        }
    }

    fun rotateIncrementUp() {
        val currentPosition = hardware.intakeRotateServo?.position
        if (currentPosition != null && currentPosition > WRIST_MIN) {
            hardware.intakeRotateServo?.position = currentPosition - WRIST_STEP
        }
    }

    fun runIntake() {
        // On a continuous servo, position of 1.0 is running forward
        hardware.intakeRotateServo?.position = 1.0
    }

    fun runEject() {
        // On a continuous servo, position of 0.0 is running reverse
        hardware.intakeRotateServo?.position = 0.0
    }

    fun stopIntake() {
        // On a continuous servo, position of 0.5 is stopped
        hardware.intakeRotateServo?.position = 0.5
    }

    companion object {
        private const val WRIST_MIN = 0.0
        private const val WRIST_MAX = 1.0
        private const val WRIST_STEP = 0.01
        //private const val EXTENSION_MIN_POSITION = 0
        //private const val EXTENSION_LOWER_BASKET_POSITION = 5
        //private const val EXTENSION_MAX_POSITION = 10

        //private const val ROTATION_SPEED = 1.0
        //private const val ROTATION_MIN_POSITION = 0
        //private const val ROTATION_MAX_POSITION = 10

        //private const val EXTENSION_KP = 0.0
        //private const val ROTATION_KP = 0.0
    }
}