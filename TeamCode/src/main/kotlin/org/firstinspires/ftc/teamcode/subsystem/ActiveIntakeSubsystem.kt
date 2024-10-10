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
        hardware.intakeRotateServo?.position = 0.33
    }

    fun rotateToBasket() {
        hardware.intakeRotateServo?.position = 0.67
    }

    fun rotateRight() {
        hardware.intakeRotateServo?.position = 1.0
    }

    fun rotateIncrementDown() {
        val currentPosition = hardware.intakeRotateServo?.position
        if (currentPosition != null && currentPosition < 1.0) {
            hardware.intakeRotateServo?.position = currentPosition + 0.1
        }
    }

    fun rotateIncrementUp() {
        val currentPosition = hardware.intakeRotateServo?.position
        if (currentPosition != null && currentPosition > 0.0) {
            hardware.intakeRotateServo?.position = currentPosition - 0.1
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
}