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
            if (gamepad1.left_trigger > 0.5 && !wasLT) {
                REmoverImpl.WKP += 0.001
            }
            if (gamepad1.right_trigger > 0.5 && !wasRT) {
                REmoverImpl.WKP -= 0.001
            }

            if (gamepad1.left_bumper && !wasLB) {
                REmoverImpl.WKD += 0.001
            }
            if (gamepad1.right_bumper && !wasRB) {
                REmoverImpl.WKD -= 0.001
            }

            if (gamepad1.x && !wasX) {
                REmoverImpl.WKI += 0.001
            }
            if (gamepad1.b && !wasB) {
                REmoverImpl.WKI -= 0.001
            }

            if (gamepad1.a && !wasA) {
                driveMachine.state = DriveMachine.State.PID2POINT
                driveMachine.removerImpl.init(
                    robotState.ppRobotPose,
                    RobotPose(12.0,-20.0,Math.PI/2),
                )
            }

            if (gamepad1.y && !wasY) {
                driveMachine.state = DriveMachine.State.PID2POINT
                driveMachine.removerImpl.init(
                    robotState.ppRobotPose,
                    RobotPose(0.0,0.0,0.0),
                )
            }

            wasLB = gamepad1.left_bumper
            wasRB = gamepad1.right_bumper
            wasLT = gamepad1.left_trigger > 0.5
            wasRT = gamepad1.right_trigger > 0.5
            wasA = gamepad1.a
            wasB = gamepad1.b
            wasX = gamepad1.x
            wasY = gamepad1.y

            gamepadState.update()
            robotState.update()
            driveMachine.update(robotState, gamepadState)

            telemetry.addData("FKP", REmoverImpl.WKP)
            telemetry.addData("FKD", REmoverImpl.WKD)
            telemetry.addData("FKI", REmoverImpl.WKI)
            telemetry.addData("x", robotState.ppX)
            telemetry.addData("y", robotState.ppY)
            telemetry.addData("theta", robotState.ppThetaRad)
            telemetry.update()
        }
    }
}