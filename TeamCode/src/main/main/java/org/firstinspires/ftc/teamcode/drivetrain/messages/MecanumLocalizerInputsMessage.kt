package org.firstinspires.ftc.teamcode.drivetrain.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

class MecanumLocalizerInputsMessage(
    leftFront: PositionVelocityPair?,
    leftBack: PositionVelocityPair?,
    rightBack: PositionVelocityPair?,
    rightFront: PositionVelocityPair?,
    angles: YawPitchRollAngles
) {
    var timestamp: Long
    var leftFront: PositionVelocityPair?
    var leftBack: PositionVelocityPair?
    var rightBack: PositionVelocityPair?
    var rightFront: PositionVelocityPair?
    var yaw: Double = 0.0
    var pitch: Double = 0.0
    var roll: Double = 0.0

    init {
        this.timestamp = System.nanoTime()
        this.leftFront = leftFront
        this.leftBack = leftBack
        this.rightBack = rightBack
        this.rightFront = rightFront
        run {
            this.yaw = angles.getYaw(AngleUnit.RADIANS)
            this.pitch = angles.getPitch(AngleUnit.RADIANS)
            this.roll = angles.getRoll(AngleUnit.RADIANS)
        }
    }
}
