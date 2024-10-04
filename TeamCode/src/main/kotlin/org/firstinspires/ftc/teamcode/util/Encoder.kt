package org.firstinspires.ftc.teamcode.util

abstract class Encoder {
    enum class Direction(val multiplier: Int) {
        FORWARD(1),
        REVERSE(-1)
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     */
    var direction: Direction = Direction.FORWARD

    abstract val currentPosition: Double
    abstract val correctedVelocity: Double

}