package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.hardware.RobotPose
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware.PoseSet
import kotlin.math.PI
import kotlin.math.atan2

class TurretImpl(private val hw: GerthaHardwareMap) {

    val poseSet = PoseSet()


    val max = 0.93

    val zeroPosition = 0.54

    var targetPose = poseSet.RIGHT_HIVE_AT


    fun setTargetHive(hiveUp: RobotPose) {
        targetPose = hiveUp
    }

    fun setToZero() {
        hw.pollenTurret.position = zeroPosition
    }

    fun setDegrees(degrees :Double) {
        var targetTicks = (degrees + 81.0) / 150.0
        if (targetTicks > 0.93) {
            targetTicks = 0.93
        }
        else if (targetTicks < 0.0) {
            targetTicks = 0.0
        }
        hw.pollenTurret.position = targetTicks
    }


    fun update (robotState: RobotState) {
        if (robotState.rightHive) {
            targetPose = poseSet.RIGHT_HIVE_AT
        }
        else {
            targetPose = poseSet.LEFT_HIVE_AT
        }
        val currentPose = robotState.ppRobotPose
        val targetDegrees = getTargetFromPinpoint(currentPose)
        var targetTicks = (targetDegrees + 81.0) / 150.0

        if (targetTicks > 0.93) {
            targetTicks = 0.93
        }
        else if (targetTicks < 0.0) {
            targetTicks = 0.0
        }

        hw.pollenTurret.position = targetTicks
    }

    fun toDeg(radians: Double): Double {
        return radians * (180.0 / PI)
    }

    private fun getTargetFromPinpoint(currentPose: RobotPose): Double {
        val x = targetPose.x - (currentPose.x)
        val y = targetPose.y - (currentPose.y)
        val robotAngle = toDeg(currentPose.a)
        val goalAngle = toDeg(atan2(y, x))

        var targetAngle = (180 + goalAngle - robotAngle)

        while (targetAngle >= 180.0) targetAngle -= 360.0
        while (targetAngle < -180.0) targetAngle += 360.0

        return(targetAngle)
    }



}