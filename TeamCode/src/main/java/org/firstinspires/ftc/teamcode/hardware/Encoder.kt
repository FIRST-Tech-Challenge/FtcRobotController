package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor

class Encoder(val motor: DcMotor) {
    @JvmField var isFlipped = false
    private val flippedFactor: Int
        get() = if (isFlipped) -1 else 1

    fun getCurrentPosition(): Int = motor.currentPosition * flippedFactor
}