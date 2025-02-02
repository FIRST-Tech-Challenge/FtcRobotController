package org.firstinspires.ftc.teamcode.hardware

import android.util.Log
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import dev.aether.collaborative_multitasking.ITask
import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.SharedResource
import dev.aether.collaborative_multitasking.TaskTemplate
import org.firstinspires.ftc.teamcode.Hardware
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.time.DurationUnit
import kotlin.time.ExperimentalTime
import kotlin.time.TimeSource
import kotlin.time.TimeSource.Monotonic.ValueTimeMark

class Ascent(
    private val leftAscent: DcMotorSimple,
    private val leftAscentEncoder: Encoder,
    private val rightAscent: DcMotorSimple,
    private val rightAscentEncoder: Encoder,
) {
    companion object {
        const val MAXIMUM = 0
        const val MINIMUM = -3400

        const val MAXPOW = 1.0
        const val CLOSE = 500.0

        const val KP = 1.0 / CLOSE
        const val KI = 0.000
    }

    private var offsetL = 0
    private var offsetR = 0

    private var targetPos: Int = 0

    val leftPosition; get() = leftAscentEncoder.getCurrentPosition() + offsetL
    val rightPosition; get() = rightAscentEncoder.getCurrentPosition() + offsetR
    val averagePosition: Double; get() = (leftPosition + rightPosition) / 2.0

    /**
     * Asserts that the current position is actually the value provided, not the currently read value from the encoders.
     * For example, at the start of TeleOp, the position of these should be nonzero if continuing from Auto.
     */
    fun calibrate(actualPosition: Int) {
        offsetL = actualPosition - leftAscentEncoder.getCurrentPosition()
        offsetR = actualPosition - rightAscentEncoder.getCurrentPosition()
    }

    private val timeSource = TimeSource.Monotonic
    private var lastTime: ValueTimeMark? = null
    private var eTotal: Double = 0.0

    // TODO: detect lack of tension and abort to prevent unspooling?
    private fun runTo(device: DcMotorSimple, currentPosition: Int) {
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
        var power = error * KP + eTotal * KI

        Log.i("Ascent", "error: %d (iE: %f) target: %d actual: %d power: %f".format(error, eTotal, targetPos, currentPosition, power))

        device.power = min(max(power, -MAXPOW), +MAXPOW)
    }

    fun update() {
        runTo(leftAscent, leftPosition)
        runTo(rightAscent, rightPosition)
    }

    fun setTargetPosition(target: Int) {
        targetPos = target
        if (targetPos < MINIMUM) targetPos = MINIMUM
        if (targetPos > MAXIMUM) targetPos = MAXIMUM
    }

    fun getTargetPosition(): Int = targetPos
}

class AscentProxy(
    scheduler: Scheduler,
    private val ascent: Ascent,
    private val initialPos: Int
) : TaskTemplate(scheduler) {
    companion object {
        private val requires = setOf(Hardware.Locks.Ascent)
        private var instCount = 0
    }

    val control = SharedResource("AscentProxy" + (++instCount))
    private val provides = setOf(control)

    override fun requirements() = requires

    override val daemon = true

    override fun invokeOnTick() {
        ascent.update()
    }

    override fun invokeOnStart() {
        ascent.calibrate(initialPos)
        ascent.setTargetPosition(initialPos)
    }

    fun target(target: Int): ITask {
        return object: TaskTemplate(scheduler) {
            override fun invokeOnStart() {
                ascent.setTargetPosition(target)
            }

            override fun invokeIsCompleted() = true
        }
    }

    @JvmOverloads
    fun moveTo(target: Int, range: Int, maxDuration: Double = 0.0): ITask {
        return if (maxDuration >= 0) object: TaskTemplate(scheduler) {
            private val t = ElapsedTime()

            override fun requirements() = provides

            override fun invokeOnStart() {
                ascent.setTargetPosition(target)
                t.reset()
            }

            override fun invokeIsCompleted(): Boolean {
                val pos = ascent.averagePosition
                if (t.time() >= maxDuration) {
                    Log.w("AscentProxy", "giving up, at %d, target %d +- %d".format(pos, target, range))
                    return true
                }
                if (abs(pos - target) < range) return true
                return false
            }
        }
        else object: TaskTemplate(scheduler) {
            override fun requirements() = provides

            override fun invokeOnStart() {
                ascent.setTargetPosition(target)
            }

            override fun invokeIsCompleted(): Boolean {
                if (abs(ascent.averagePosition - target) < range) return true
                return false
            }
        }
    }
}
