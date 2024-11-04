package org.firstinspires.ftc.teamcode.mmooover.kinematics

import java.io.DataInputStream
import java.io.DataOutputStream
import kotlin.math.abs


private const val EPSILON = 1e-6

interface Timed {
    val t: Double
}

abstract class Waypoint {
    abstract val x: Double
    abstract val y: Double

    override fun equals(other: Any?) = when (other) {
        null -> false
        !is Waypoint -> false
        else -> other.x == this.x && other.y == this.y
    }

    open infix fun approx(other: Any?) = when (other) {
        null -> false
        !is Waypoint -> false
        else -> abs(other.x - this.x) < EPSILON && abs(other.y - this.y) < EPSILON
    }

    override fun hashCode(): Int {
        // picked semi-randomly from the Wikipedia list of prime numbers
        return (2687 * x.hashCode()) xor (5521 * y.hashCode())
    }
}

abstract class TimedWaypoint: Timed, Waypoint()

abstract class Waypoint3 : Waypoint() {
    abstract val r: Double

    override fun equals(other: Any?) = when (other) {
        null -> false
        !is Waypoint3 -> false
        else -> super.equals(other) && this.r == other.r
    }

    override infix fun approx(other: Any?) = when (other) {
        null -> false
        !is Waypoint3 -> false
        else -> super.approx(other) && abs(other.r - this.r) < EPSILON
    }

    override fun hashCode(): Int = super.hashCode() xor 6899 * r.hashCode()
}
abstract class TimedWaypoint3: Timed, Waypoint3()

data class MinimalWaypoint(override val x: Double, override val y: Double) : Waypoint()
data class
Minimal3Waypoint(override val x: Double, override val y: Double, override val r: Double) :
    Waypoint3()

object WaypointSerializer {
    fun <T : Waypoint> serialize2(waypoints: List<T>, target: DataOutputStream) {
        target.writeBytes("WP")
        target.writeByte(2)
        target.writeInt(waypoints.size)
        val expectedSize = waypoints.size
        for ((i, value) in waypoints.withIndex()) {
            if (i >= expectedSize) throw IndexOutOfBoundsException("Too many values since writing size!")
            target.writeDouble(value.x)
            target.writeDouble(value.y)
        }
    }

    fun <T : Waypoint3> serialize3(waypoints: List<T>, target: DataOutputStream) {
        target.writeBytes("WP")
        target.writeByte(3)
        target.writeInt(waypoints.size)
        val expectedSize = waypoints.size
        for ((i, value) in waypoints.withIndex()) {
            if (i >= expectedSize) throw IndexOutOfBoundsException("Too many values since writing size!")
            target.writeDouble(value.x)
            target.writeDouble(value.y)
            target.writeDouble(value.r)
        }
    }

    @JvmOverloads
    fun deserialize2(target: DataInputStream, withHeaders: Boolean = true): MutableList<Waypoint> {
        if (withHeaders) {
            assert(
                target.readByte() == 'W'.code.toByte()
                        && target.readByte() == 'P'.code.toByte()
            ) { "Not a waypoint file (invalid magic)" }
            val channels = target.readByte().toInt()
            assert(
                channels == 2
            ) { "Waypoint file has wrong number of channels ($channels, expected 2)" }
        }
        val count = target.readInt()
        val output: MutableList<Waypoint> = mutableListOf()
        repeat(count) {
            output.add(MinimalWaypoint(target.readDouble(), target.readDouble()))
        }
        return output
    }

    @JvmOverloads
    fun deserialize3(target: DataInputStream, withHeaders: Boolean = true): MutableList<Waypoint3> {
        if (withHeaders) {
            assert(
                target.readByte() == 'W'.code.toByte()
                        && target.readByte() == 'P'.code.toByte()
            ) { "Not a waypoint file (invalid magic)" }
            val channels = target.readByte().toInt()
            assert(
                channels == 3
            ) { "Waypoint file has wrong number of channels ($channels, expected 3)" }
        }
        val count = target.readInt()
        val output: MutableList<Waypoint3> = mutableListOf()
        repeat(count) {
            output.add(Minimal3Waypoint(
                target.readDouble(),
                target.readDouble(),
                target.readDouble()
            ))
        }
        return output
    }

    fun deserializeAuto(target: DataInputStream): MutableList<out Waypoint> {
        assert(
            target.readByte() == 'W'.code.toByte()
                    && target.readByte() == 'P'.code.toByte()
        ) { "Not a waypoint file (invalid magic)" }
        val channels = target.readByte().toInt()
        return when (channels) {
            2 -> deserialize2(target, false)
            3 -> deserialize3(target, false)
            else -> throw IllegalArgumentException("File contains an invalid number of channels ($channels), expected 2 or 3")
        }
    }
}