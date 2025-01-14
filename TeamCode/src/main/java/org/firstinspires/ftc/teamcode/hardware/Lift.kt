package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Hardware
import kotlin.math.max
import kotlin.math.min

private fun clamp(a: Int, low: Int, hi: Int): Int {
    return max(low, min(a, hi))
}
private fun clamp(a: Double, low: Double, hi: Double): Double {
    return max(low, min(a, hi))
}


@Suppress("unused")
class Lift(val primaryMotor: DcMotor, val secondaryMotor: DcMotor) {
    companion object {
        const val MAX_HEIGHT = Hardware.VLIFT_MAX_HEIGHT
        const val MIN_HEIGHT = 0
        const val POWEROFF_ZERO = Hardware.VLIFT_POWEROFF_HEIGHT;

        const val KP = 1.0 / Hardware.VLIFT_CLOSENESS
        const val KI = .0001 // maybe
        const val KD = 0.0
    }
    val currentPosition; get() = primaryMotor.currentPosition
    private var power: Double = 0.0
    private var targetPos: Int = 0

    fun stop() {
        setPower(0.0)
    }

    fun setPower(newPower: Double) {
        if (newPower > 0 && currentPosition >= MAX_HEIGHT) return stop()
        if (newPower < 0 && currentPosition <= MIN_HEIGHT) return stop()
        primaryMotor.power = newPower
        secondaryMotor.power = newPower
        this.power = newPower
    }

    fun getPower() = power
    fun getTargetPosition() = targetPos

    fun setTargetPosition(newPos: Int) {
        targetPos = clamp(newPos, MIN_HEIGHT, MAX_HEIGHT)
    }

    init {
        setPower(0.0)
        primaryMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        primaryMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        secondaryMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        targetPos = 0
    }

    fun update() {
        val error = targetPos - currentPosition
        var power = error * KP /* + KI + KD */
        if (power < -0.05) power /= 10.0
        val powerFinal = clamp(power, -0.1, 1.0)

        if (targetPos <= 0 && currentPosition <= POWEROFF_ZERO) setPower(0.0)
        else setPower(powerFinal)
    }
}