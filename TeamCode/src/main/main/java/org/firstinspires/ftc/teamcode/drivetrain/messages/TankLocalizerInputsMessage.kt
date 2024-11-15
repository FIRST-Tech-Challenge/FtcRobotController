package org.firstinspires.ftc.teamcode.drivetrain.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair

class TankLocalizerInputsMessage(
    left: MutableList<PositionVelocityPair?>,
    right: MutableList<PositionVelocityPair?>
) {
    var timestamp: Long
    var left: Array<PositionVelocityPair?>?
    var right: Array<PositionVelocityPair?>?

    init {
        this.timestamp = System.nanoTime()
        this.left = left.toTypedArray<PositionVelocityPair?>()
        this.right = right.toTypedArray<PositionVelocityPair?>()
    }
}
