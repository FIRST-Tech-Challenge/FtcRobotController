package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.RobotLog
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.SharedResource
import dev.aether.collaborative_multitasking.TaskTemplate
import org.firstinspires.ftc.teamcode.Hardware
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.time.DurationUnit
import kotlin.time.TimeSource
import kotlin.time.TimeSource.Monotonic.ValueTimeMark

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
        const val POWEROFF_ZERO = Hardware.VLIFT_POWEROFF_HEIGHT
        const val CLOSE = Hardware.VLIFT_CLOSENESS

        const val KP = 1.0 / CLOSE
        const val KI = .15 // maybe
        const val KD = 0.0
    }
    val currentPosition; get() = primaryMotor.currentPosition
    private var power: Double = 0.0
    private var targetPos: Int = 0

    private var isDisabled: Boolean = false

    fun disable() {
        isDisabled = true
        setPower(0.0)
        primaryMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        secondaryMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        RobotLog.addGlobalWarningMessage("The lift is disabled!")
    }

    fun stop() {
        setPower(0.0)
    }

    private fun setPower(newPower: Double) {
        if (newPower > 0 && currentPosition >= MAX_HEIGHT) return stop()
        if (newPower < 0 && currentPosition <= MIN_HEIGHT) return stop()
        primaryMotor.power = newPower
        secondaryMotor.power = newPower
        this.power = newPower
    }

    fun getPower() = power
    fun getTargetPosition() = targetPos

    var eTotal = 0.0
        private set

    fun setTargetPosition(newPos: Int) {
        targetPos = clamp(newPos, MIN_HEIGHT, MAX_HEIGHT)
        eTotal = 0.0
    }

    init {
        setPower(0.0)
        primaryMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        primaryMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        secondaryMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        targetPos = 0
    }

    private val timeSource = TimeSource.Monotonic
    private var lastTime: ValueTimeMark? = null

    private fun runToPosition() {
        val error = targetPos - currentPosition
        if (lastTime != null) {
            val now = timeSource.markNow()
            val dt = (now - lastTime!!).toDouble(DurationUnit.SECONDS)
            lastTime = now
            if (abs(error) <= CLOSE) {
                eTotal += error * dt
            } else {
                eTotal = 0.0
            }
        } else lastTime = timeSource.markNow()
        var power = error * KP + eTotal * KI /* +D */
        if (power < -0.05) power /= 10.0
        val powerFinal = clamp(power, -0.02, 1.0)

        if (targetPos <= 0 && currentPosition <= POWEROFF_ZERO) setPower(0.0)
        else setPower(powerFinal)
    }

    fun update() {
        if (!isDisabled) runToPosition()
    }
}
