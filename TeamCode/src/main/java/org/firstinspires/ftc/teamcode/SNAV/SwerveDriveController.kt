package org.firstinspires.ftc.teamcode.SNAV

import org.firstinspires.ftc.teamcode.SNAV.Util.Angle
import java.lang.Math.toDegrees
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

class SwerveDriveController(val leftModule: SwerveModule, val rightModule: SwerveModule) {
    var xSpeedPower: Double = 0.0 // in/s
    var ySpeedPower: Double = 0.0 // in/s
    var rotationPower: Double = 0.0 // Rad/s

    var leftAngle: Double = 0.0 // in/s
    var rightAngle: Double = 0.0 // Degrees
    var leftSpeed: Double = 0.0 // in/s
    var rightSpeed: Double = 0.0 // Degrees

    fun updateModules() {
        leftSpeed = sqrt(xSpeedPower.pow(2) + (ySpeedPower - ((rotationPower * leftModule.distanceFromCenter) / 2)).pow(2))
        rightSpeed = sqrt(xSpeedPower.pow(2) - (ySpeedPower - ((rotationPower * rightModule.distanceFromCenter) / 2)).pow(2))

        leftAngle = toDegrees(atan2(ySpeedPower + (rotationPower * leftModule.distanceFromCenter) / 2, xSpeedPower))
        rightAngle = toDegrees(atan2(ySpeedPower - (rotationPower * leftModule.distanceFromCenter) / 2, xSpeedPower))

        leftModule.setTranslationalSpeed(leftSpeed)
        rightModule.setTranslationalSpeed(rightSpeed)

        leftModule.setTargetAngle(Angle(leftAngle))
        rightModule.setTargetAngle(Angle(rightAngle))
    }
}