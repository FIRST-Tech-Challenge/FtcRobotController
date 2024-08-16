package com.millburnx.purePursuit
import kotlin.math.abs

class Robot(size: Point, val lookahead: Double = 14.0) {
    var position = Point(0.0, 0.0)
    var heading = 0.0
    var speed = 12.0 // inches per second
    var turnRate = Math.toRadians(180.0) // radians per second

    var power = Point(0.0, 0.0)
    var anglePower = 0.0

    val lookaheadCircle: Circle
        get() = Circle(position, lookahead)

    fun drive(x: Double, y: Double, rx: Double) {
        val denominator: Double = (abs(y) + abs(x) + abs(rx)).coerceAtLeast(1.0)

        val speed = this.speed
        val turnRate = this.turnRate

        val scaled = Point(x, y) / denominator * speed
        val scaledRX = rx / denominator * turnRate

        power = scaled
        anglePower = scaledRX
    }

    fun update(dt: Double) {
        val rotated = power.rotate(heading)
        position += rotated * dt
        heading = Utils.normalizeAngle(heading + anglePower * dt)
    }
}