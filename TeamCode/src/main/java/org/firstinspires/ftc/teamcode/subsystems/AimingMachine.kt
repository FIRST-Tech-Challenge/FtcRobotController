package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.*
import org.firstinspires.ftc.teamcode.ShooterImpl
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware.PoseSet
import kotlin.math.abs
import kotlin.math.tan

class AimingMachine(private val hw: GerthaHardwareMap, private val poseSet: PoseSet) {

    enum class State {
        FULL,
        NO_TURRET,

        NO_NOTHING,
        AUTO
    }

    lateinit var state: State

    lateinit var prevState: State

    var justTransitioned = true

    val shooterImpl = ShooterImpl(hw)
    val turretImpl = TurretImpl(hw, poseSet)

    var constantVel = 0.0

    fun setHardcode(vel: Double) {
        constantVel = vel
    }

    fun init() {
        state = State.FULL
        prevState = State.FULL
        turretImpl.targetPose = poseSet.LEFT_HIVE_AT
        shooterImpl.setTarget(AUTO_SHOOT_SPEED)
    }

    fun update (robotState: RobotState, input: GamepadState) {

        justTransitioned = prevState != state
        prevState = state
        val horizontalDistance = distance(robotState, poseSet) * 0.0254
        val top = (-9.8) * (horizontalDistance * horizontalDistance)
        val bottom = (2 * Math.cos(ANGLE_OF_SHOT) * Math.cos(ANGLE_OF_SHOT) * (DISTANCE_TO_TOP - (horizontalDistance * tan(ANGLE_OF_SHOT))))
        val velMps = Math.sqrt(top / bottom)
        // 126.25m 474.5b
//        val velShooter = (126.25 * velMps) + 474.5
        var velShooter = (92.23 * velMps) + 283.4
        if (abs(bottom) < 0.001){
            velShooter = 800.0
        }
        val diffOff20 = velShooter % 20

        if (diffOff20 <= 10){
            velShooter -= diffOff20
        }
        else {
            velShooter += (20 - diffOff20)
        }
        Log.i("distance", horizontalDistance.toString())
        Log.i("top", top.toString())
        Log.i("bottom", bottom.toString())
        Log.i("Target", velShooter.toString())

        when (state) {
            State.FULL -> {
                turretImpl.update(robotState)
                shooterImpl.setTarget(velShooter)
                shooterImpl.tickShooter()

                if (input.start2) {
                    state = State.NO_TURRET
                }
            }

            State.NO_TURRET -> {
                if (justTransitioned) {
                    turretImpl.setToZero()
                }
                shooterImpl.setTarget(velShooter)
                shooterImpl.tickShooter()

                if (input.start2) {
                    state = State.NO_NOTHING
                }
            }
            State.NO_NOTHING -> {
                if (justTransitioned) {
                    turretImpl.setToZero()
                }
                shooterImpl.setTarget(AUTO_SHOOT_SPEED)
                shooterImpl.tickShooter()

                if (input.start2) {
                    state = State.FULL
                }
            }
            State.AUTO -> {
                if (justTransitioned) {
                    turretImpl.setToZero()
                }
//                turretImpl.update(robotState)
                shooterImpl.setTarget(constantVel)
                shooterImpl.tickShooter()
//                if (robotState.ppX > 0) {
//                    turretImpl.setDegrees(15.0)
//                }
//                else {
//                    turretImpl.setDegrees(-15.0)
//                }
            }
        }
    }
}
