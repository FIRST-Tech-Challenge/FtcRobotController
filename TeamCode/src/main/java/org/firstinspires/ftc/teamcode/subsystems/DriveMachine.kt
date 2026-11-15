package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.times

class DriveMachine(private val hw: GerthaHardwareMap) {
    enum class State {
        FIELD_CENTRIC,
        PID2POINT,
    }

    lateinit var state: State

    val removerImpl = REmoverImpl()

    var skew = 0.0

    fun init(robotState: RobotState) {
        if (!robotState.isRed) {
            skew = PI
        }
        state = State.FIELD_CENTRIC
    }

    fun update(robotState: RobotState, input: GamepadState) {


        when (state) {
            State.FIELD_CENTRIC -> {
                val angle = -(robotState.ppThetaRad + Math.PI/2 + skew)

                var rotX = input.jx1 * cos(angle) - input.jy1 * sin(angle)
                val rotY = input.jx1 * sin(angle) + input.jy1 * cos(angle)

                rotX *= 1.1 // Counteract imperfect strafing

                val denominator = max(abs(rotY) + abs(rotX) + abs(input.rx1), 1.0)
                val frontLeftPower = (rotY + rotX + input.rx1) / denominator
                val backLeftPower = (rotY - rotX + input.rx1) / denominator
                val frontRightPower = (rotY - rotX - input.rx1) / denominator
                val backRightPower = (rotY + rotX - input.rx1) / denominator

                hw.frontLeft.power = frontLeftPower
                hw.backLeft.power = backLeftPower
                hw.frontRight.power = frontRightPower
                hw.backRight.power = backRightPower
            }
            State.PID2POINT -> {
                removerImpl.tick(
                    robotState.ppX,
                    robotState.ppY,
                    robotState.ppThetaRad,
                    robotState.velXInch,
                    robotState.velYInch,
                    robotState.angVelRad,
                    robotState.velInch,
                    hw
                )
            }
        }
    }

}