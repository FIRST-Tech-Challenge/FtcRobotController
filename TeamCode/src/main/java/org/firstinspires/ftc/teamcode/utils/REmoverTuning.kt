package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.hardware.RobotPose
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.DriveMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.GamepadState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.REmoverImpl
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RobotState

@TeleOp
class REmoverTuning: LinearOpMode() {
    lateinit var hw: GerthaHardwareMap
    override fun runOpMode() {
        hw = GerthaHardwareMap(hardwareMap)
        val robotState = RobotState(hw)
        val gamepadState = GamepadState(gamepad1, gamepad2)
        val driveMachine = DriveMachine(hw)

        waitForStart()

        driveMachine.init(robotState)
        robotState.init(RobotPose(0.0, 0.0, 0.0))

        var wasLT = false
        var wasRT = false
        var wasLB = false
        var wasRB = false
        var wasX = false
        var wasB = false
        var wasA = false
        var wasY = false

        while (opModeIsActive()) {
            if (gamepadState.lt1) {
                REmoverImpl.SKP += 0.001
            }
            if (gamepadState.rt1) {
                REmoverImpl.SKP -= 0.001
            }

            if (gamepadState.lb1) {
                REmoverImpl.SKD += 0.001
            }
            if (gamepadState.rb1) {
                REmoverImpl.SKD -= 0.001
            }

            if (gamepadState.a1) {
                REmoverImpl.SKI += 0.0001
            }
            if (gamepadState.b1) {
                REmoverImpl.SKI -= 0.0001
            }

            if (gamepadState.start1) {
                driveMachine.state = DriveMachine.State.PID2POINT
                driveMachine.removerImpl.init(
                    robotState.ppRobotPose,
                    RobotPose(0.0,24.0,0.0),
                )
            }

            if (gamepadState.jx1>0.5) {
                driveMachine.state = DriveMachine.State.PID2POINT
                driveMachine.removerImpl.init(
                    robotState.ppRobotPose,
                    RobotPose(0.0,0.0,0.0),
                )
            }

            gamepadState.update()
            robotState.update()
            driveMachine.update(robotState, gamepadState)

            telemetry.addData("FKP", REmoverImpl.SKP)
            telemetry.addData("FKD", REmoverImpl.SKD)
            telemetry.addData("FKI", REmoverImpl.SKI)
            telemetry.addData("x", robotState.ppX)
            telemetry.addData("y", robotState.ppY)
            telemetry.addData("theta", robotState.ppThetaRad)
            telemetry.update()
        }
    }
}