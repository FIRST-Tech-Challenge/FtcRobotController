package com.millburnx.purepursuit

import com.millburnx.utils.Circle
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import kotlin.math.abs

class Robot(size: Vec2d, val lookahead: Double = 14.0) {
    var position = Vec2d(0.0, 0.0)
    var heading = 0.0
    private var speed = 12.0 // inches per second
    private var turnRate = Math.toRadians(180.0) // radians per second

    private var power = Vec2d(0.0, 0.0)
    private var anglePower = 0.0

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

    fun toPair(): Pair<Vec2d, Double> = position to heading
}