package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.hardware.RobotPose
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware.PoseSet
import kotlin.math.PI
import kotlin.math.atan2

class TurretImpl(private val hw: GerthaHardwareMap, private val poseSet: PoseSet) {


    val max = 1.0

    val zeroPosition = 0.47

    var targetPose = poseSet.RIGHT_HIVE_AT


    fun setTargetHive(hiveUp: RobotPose) {
        targetPose = hiveUp
    }

    fun setToZero() {
        hw.pollenTurret.position = zeroPosition
    }

    fun setDegrees(degrees :Double) {
        var targetTicks = (degrees + 76.5) / 150.0
        if (targetTicks > 0.93) {
            targetTicks = 0.93
        }
        else if (targetTicks < 0.0) {
            targetTicks = 0.0
        }
        hw.pollenTurret.position = targetTicks
    }


    fun update (robotState: RobotState) {
        Log.i("turret", targetPose.toString())
        if (robotState.rightHive) {
            targetPose = poseSet.RIGHT_HIVE_AT.copy()
        }
        else {
            targetPose = poseSet.LEFT_HIVE_AT.copy()
        }
        if (robotState.ppY > 36) {
            targetPose.y += 10.0
        }
        else if (robotState.ppY < -36) {
            targetPose.y -= 3.0
        }
        val currentPose = robotState.ppRobotPose
        val targetDegrees = getTargetFromPinpoint(currentPose)
        var targetTicks = (targetDegrees + (zeroPosition * 1.5 * 100)) / 150.0

        if (targetTicks > max) {
            targetTicks = max
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