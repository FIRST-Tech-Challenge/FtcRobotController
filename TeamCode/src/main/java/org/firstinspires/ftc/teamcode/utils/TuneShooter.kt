package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.ShooterImpl
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.DriveMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.GamepadState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.REmoverImpl
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RobotState
@TeleOp
class TuneShooter: LinearOpMode() {

    lateinit var hw: GerthaHardwareMap
    override fun runOpMode() {
        hw = GerthaHardwareMap(hardwareMap)
        val shooterImpl = ShooterImpl(hw)

        waitForStart()

        var wasLT = false
        var wasRT = false
        var wasLB = false
        var wasRB = false
        var wasX = false
        var wasB = false
        var wasA = false
        var wasY = false

        shooterImpl.setTarget(800.0)

        val gamepadState = GamepadState(gamepad1, gamepad2)

        while (opModeIsActive()) {
            if (gamepadState.lb1) {
                ShooterImpl.KP += 0.001
            }
            if (gamepadState.rb1) {
                ShooterImpl.KP -= 0.001
            }

            if (gamepadState.lt1) {
                ShooterImpl.KD += 0.001
            }
            if (gamepadState.rt1) {
                ShooterImpl.KD -= 0.001
            }

            if (gamepadState.a1) {
                ShooterImpl.KI += 0.001
            }
            if (gamepadState.b1) {
                ShooterImpl.KI -= 0.001
            }

            shooterImpl.tickShooter()
            telemetry.addData("kf", shooterImpl.f)
            telemetry.addData("kp", ShooterImpl.KP)
            telemetry.addData("kd", ShooterImpl.KD)
            telemetry.addData("ki", ShooterImpl.KI)
            telemetry.addData("vel", hw.pollenShooter.velocity)
            telemetry.addData("power", hw.pollenShooter.power)
            telemetry.update()

            gamepadState.update()
        }
    }
}