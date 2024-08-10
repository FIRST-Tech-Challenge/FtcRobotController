package com.acmerobotics.roadrunner.ftc

import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.max
import kotlin.math.min
import kotlin.math.round

class PositionVelocityPair(
    @JvmField val position: Int, @JvmField val velocity: Int,
    @JvmField val rawPosition: Int, @JvmField val rawVelocity: Int
)

sealed interface Encoder {
    fun getPositionAndVelocity(): PositionVelocityPair

    val controller: DcMotorController
    var direction: DcMotorSimple.Direction
}

class RawEncoder(private val m: DcMotorEx) : Encoder {
    override var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

    private fun applyDirection(x: Int): Int {
        var x = x
        if (m.direction == DcMotorSimple.Direction.REVERSE) {
            x = -x
        }

        if (direction == DcMotorSimple.Direction.REVERSE) {
            x = -x
        }

        return x
    }

    override fun getPositionAndVelocity(): PositionVelocityPair {
        val rawPosition = m.currentPosition
        val rawVelocity = m.velocity.toInt()
        return PositionVelocityPair(
            applyDirection(rawPosition),
            applyDirection(rawVelocity),
            rawPosition,
            rawVelocity,
        )
    }

    override val controller: DcMotorController
        get() = m.controller
}

class RollingThreeMedian {
    private val history = DoubleArray(3)
    private var i: Int = 0

    fun update(x: Double): Double {
        history[i] = x

        i = (i + 1) % 3

        return max(
            min(history[0], history[1]),
            min(max(history[0], history[1]), history[2]))
    }
}

// encoder velocities are sent as 16-bit ints
// by the time they reach here, they are widened into an int and possibly negated
private const val CPS_STEP = 0x10000

private fun inverseOverflow(input: Int, estimate: Double): Int {
    // convert to uint16
    var real = input and 0xFFFF
    // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
    // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
    real += real % 20 / 4 * CPS_STEP
    // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
    real += round((estimate - real) / (5 * CPS_STEP)).toInt() * 5 * CPS_STEP
    return real
}

class OverflowEncoder(@JvmField val encoder: RawEncoder) : Encoder {
    private var lastPosition: Int = encoder.getPositionAndVelocity().position
    private val lastUpdate = ElapsedTime()

    private val velEstimate = RollingThreeMedian()

    override fun getPositionAndVelocity(): PositionVelocityPair {
        val p = encoder.getPositionAndVelocity()
        val dt = lastUpdate.seconds()
        val v = velEstimate.update((p.position - lastPosition) / dt)

        lastPosition = p.position
        lastUpdate.reset()

        return PositionVelocityPair(
            p.position,
            inverseOverflow(p.velocity, v),
            p.rawPosition,
            p.rawVelocity,
        )
    }

    override val controller: DcMotorController
        get() = encoder.controller

    override var direction: DcMotorSimple.Direction
        get() = encoder.direction
        set(value) {
            encoder.direction = value
        }
}
