package org.firstinspires.ftc.teamcode.util.math

data class Vector(val mag: Double, val angle: Angle) {
    constructor(ijform: Point) : this(ijform.hypot, ijform.atan2)

    val ijform get() = Point(mag * angle.cos, mag * angle.sin)

    val norm: Vector get() = Vector(ijform / mag)

    operator fun plus(a: Vector) = Vector(a.ijform + ijform)
    operator fun minus(a: Vector) = Vector(a.ijform - ijform)
    operator fun times(scalar: Double) = Vector(mag * scalar, angle)
    operator fun div(scalar: Double) = this * (1 / scalar)
}
