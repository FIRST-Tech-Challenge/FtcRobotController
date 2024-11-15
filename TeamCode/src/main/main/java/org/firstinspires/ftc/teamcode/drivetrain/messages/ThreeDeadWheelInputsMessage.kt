package org.firstinspires.ftc.teamcode.drivetrain.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair

class ThreeDeadWheelInputsMessage(
    par0: PositionVelocityPair?,
    par1: PositionVelocityPair?,
    perp: PositionVelocityPair?
) {
    var timestamp: Long
    var par0: PositionVelocityPair?
    var par1: PositionVelocityPair?
    var perp: PositionVelocityPair?

    init {
        this.timestamp = System.nanoTime()
        this.par0 = par0
        this.par1 = par1
        this.perp = perp
    }
}