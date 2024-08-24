package org.firstinspires.ftc.teamcode.SNAV

import org.firstinspires.ftc.teamcode.SNAV.Util.Angle
import kotlin.math.cos
import kotlin.math.sin
import java.lang.Math.toDegrees
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

class SwerveDriveController(val leftModule: SwerveModule, val rightModule: SwerveModule) {
    var xSpeedPower: Double = 0.0 // in/s
    var ySpeedPower: Double = 0.0 // in/s
    var rotationPower: Double = 0.0 // Rad/s

    private var leftAngle: Double = 0.0 // in/s
    private var rightAngle: Double = 0.0 // Degrees
    private var leftSpeed: Double = 0.0 // in/s
    private var rightSpeed: Double = 0.0 // Degrees

    fun updateModules(currentRobotAngle: Double? = null, global: Boolean = true) {
        if (!global || currentRobotAngle == null) {
            leftSpeed = sqrt(xSpeedPower.pow(2) + (ySpeedPower - ((rotationPower * leftModule.distanceFromCenter) / 2)).pow(2))
            rightSpeed = sqrt(xSpeedPower.pow(2) - (ySpeedPower - ((rotationPower * rightModule.distanceFromCenter) / 2)).pow(2))

            leftAngle = toDegrees(atan2(ySpeedPower + (rotationPower * leftModule.distanceFromCenter) / 2, xSpeedPower))
            rightAngle = toDegrees(atan2(ySpeedPower - (rotationPower * leftModule.distanceFromCenter) / 2, xSpeedPower))
        } else {
            val localPowers: Array<Double> = getLocalSpeedPowers(currentRobotAngle)
            val localXSpeedPower = localPowers[0]
            val localYSpeedPower = localPowers[1]

            leftSpeed = sqrt((localXSpeedPower + rotationPower * (leftModule.distanceFromCenter / 2)).pow(2) + localYSpeedPower.pow(2))
            rightSpeed = sqrt((localXSpeedPower - rotationPower * (rightModule.distanceFromCenter / 2)).pow(2) + localYSpeedPower.pow(2))

            leftAngle = atan2(localYSpeedPower + rotationPower * (leftModule.distanceFromCenter / 2), localXSpeedPower)
            rightAngle = atan2(localYSpeedPower - rotationPower * (leftModule.distanceFromCenter / 2), localXSpeedPower)
        }

        leftModule.setTranslationalSpeed(leftSpeed)
        rightModule.setTranslationalSpeed(rightSpeed)

        leftModule.targetAngle = Angle(leftAngle)
        rightModule.targetAngle = Angle(rightAngle)
        leftModule.updateAngle()
        rightModule.updateAngle()
    }

    private fun getLocalSpeedPowers(currentRobotAngle: Double): Array<Double> {
        val speed: Array<Double> = arrayOf(0.0)
        speed[0] = xSpeedPower * cos(currentRobotAngle) + ySpeedPower * sin(currentRobotAngle)
        speed[1] = -xSpeedPower * cos(currentRobotAngle) + ySpeedPower * cos(currentRobotAngle)
        return speed
    }
}