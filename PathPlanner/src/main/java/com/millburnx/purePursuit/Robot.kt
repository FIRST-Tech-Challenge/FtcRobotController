package com.millburnx.purePursuit

import com.millburnx.purePursuit.Utils.Circle
import com.millburnx.purePursuit.Utils.Vec2d
import com.millburnx.purePursuit.Utils.Utils
import kotlin.math.abs

class Robot(size: Vec2d, val lookahead: Double = 14.0) {
    var position = Vec2d(0.0, 0.0)
    var heading = 0.0
    var speed = 12.0 // inches per second
    var turnRate = Math.toRadians(180.0) // radians per second

    var power = Vec2d(0.0, 0.0)
    var anglePower = 0.0

    val lookaheadCircle: Circle
        get() = Circle(position, lookahead)

    /**
     * Sets the motor powers of the robot
     * @param x the forward power
     * @param y the strafe power
     * @param rx the turn power
     */
    fun drive(x: Double, y: Double, rx: Double) {
        val denominator: Double = (abs(y) + abs(x) + abs(rx)).coerceAtLeast(1.0)

        val speed = this.speed
        val turnRate = this.turnRate

        val scaled = Vec2d(x, y) / denominator * speed
        val scaledRX = rx / denominator * turnRate

        power = scaled
        anglePower = scaledRX
    }

    /**
     * Updates the robot's position and heading
     * @param dt the time since the last update in seconds; Delta Time
     */
    fun update(dt: Double) {
        val rotated = power.rotate(heading)
        position += rotated * dt
        heading = Utils.normalizeAngle(heading + anglePower * dt)
    }
}