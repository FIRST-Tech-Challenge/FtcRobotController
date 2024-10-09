package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.teamcode.hardware.HardwareManager

class GripperSubsystem(
    private val hardware: HardwareManager,
    private val telemetry: MultipleTelemetry? = null
) : SubsystemBase() {
    fun setPickupHeight() {
        // On a continuous servo, position of 0.0 is running reverse.
        // We want to start reversing until we hit the home sensor.
        if (hardware.gripperHomeSensor?.isPressed == true) return

        hardware.gripperServo?.position = 0.0

        // TODO: How do we stop this in the right position?
    }

    fun setHighChamberHeight() {
        // On a continuous servo, position of 0.1 is running forward.
        // We want to start running until we reach the delivery height.
        hardware.gripperServo?.position = 1.0

        // TODO: How do we stop this in the right position?
    }

    fun setLowChamberHeight() {
        // On a continuous servo, position of 0.1 is running forward.
        // We want to start running until we reach the delivery height.
        hardware.gripperServo?.position = 1.0

        // TODO: How do we stop this in the right position?
    }
}