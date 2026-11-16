package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.*
import org.firstinspires.ftc.teamcode.ShooterImpl
import org.firstinspires.ftc.teamcode.hardware.RobotPose
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware.PoseSet

class AimingMachine(private val hw: GerthaHardwareMap) {

    enum class State {
        FULL,
        NO_TURRET
    }

    lateinit var state: State

    lateinit var prevState: State

    var justTransitioned = true

    val shooterImpl = ShooterImpl(hw)
    val turretImpl = TurretImpl(hw)
    val poseSet = PoseSet()

    fun init() {
        state = State.FULL
        prevState = State.FULL
        turretImpl.targetPose = poseSet.LEFT_HIVE_AT
        shooterImpl.setTarget(SHOOT_SPEED)
    }

    fun update (robotState: RobotState, input: GamepadState) {

        justTransitioned = prevState != state
        prevState = state

        when (state) {
            State.FULL -> {
                turretImpl.update(robotState)
                shooterImpl.setTarget(SHOOT_SPEED)
                shooterImpl.tickShooter()

                if (input.start2) {
                    state = State.NO_TURRET
                }
            }

            State.NO_TURRET -> {
                if (justTransitioned) {
                    turretImpl.setToZero()
                }
                shooterImpl.setTarget(SHOOT_SPEED)
                shooterImpl.tickShooter()

                if (input.start2) {
                    state = State.FULL
                }
            }
        }
    }
}
