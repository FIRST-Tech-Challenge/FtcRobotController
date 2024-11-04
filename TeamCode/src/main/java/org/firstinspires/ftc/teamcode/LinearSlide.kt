package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.clampi

class LinearSlide(val motor: DcMotor, private val ppr: Double, private val ratio: Double) {
    // Ratio is rotations/inch
    private val zeroPosition: Int = motor.currentPosition
    private val maxPosition: Int = motor.currentPosition + inchesToTicks(16.0)

    init {
        motor.targetPosition = motor.currentPosition
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.power = 0.25
    }

    fun extend(delta: Double) {
        motor.targetPosition = clampi(inchesToTicks(delta) + motor.currentPosition, zeroPosition, maxPosition)
    }

    fun returnToZero() {
        motor.targetPosition = zeroPosition
    }

    private fun inchesToTicks(inches: Double): Int {
        return (inches / ratio * ppr).toInt()
    }

    private fun ticksToInches(ticks: Double): Double {
        return ticks / ppr * ratio
    }
}