package org.firstinspires.ftc.teamcode.drivetrain.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

class TwoDeadWheelInputsMessage(
    par: PositionVelocityPair?,
    perp: PositionVelocityPair?,
    angles: YawPitchRollAngles,
    angularVelocity: AngularVelocity
) {
    var timestamp: Long
    var par: PositionVelocityPair?
    var perp: PositionVelocityPair?
    var yaw: Double = 0.0
    var pitch: Double = 0.0
    var roll: Double = 0.0
    var xRotationRate: Double = 0.0
    var yRotationRate: Double = 0.0
    var zRotationRate: Double = 0.0

    init {
        this.timestamp = System.nanoTime()
        this.par = par
        this.perp = perp
        run {
            this.yaw = angles.getYaw(AngleUnit.RADIANS)
            this.pitch = angles.getPitch(AngleUnit.RADIANS)
            this.roll = angles.getRoll(AngleUnit.RADIANS)
        }
        run {
            this.xRotationRate = angularVelocity.xRotationRate.toDouble()
            this.yRotationRate = angularVelocity.yRotationRate.toDouble()
            this.zRotationRate = angularVelocity.zRotationRate.toDouble()
        }
    }
}
