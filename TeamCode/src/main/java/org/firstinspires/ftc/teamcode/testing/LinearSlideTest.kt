package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.LinearSlide
import org.firstinspires.ftc.teamcode.util.GamepadState

@TeleOp(name="LinearSlideTest")
class LinearSlideTest : LinearOpMode() {
    override fun runOpMode() {
        val motor: DcMotor = hardwareMap.get(DcMotor::class.java, "one")
        val linearSlide = LinearSlide(motor, 537.7, 4.75)

        val dashboard: FtcDashboard? = FtcDashboard.getInstance()
        val dashboardTelemetry = dashboard!!.telemetry

        var pastGamepad1 = GamepadState()
        pastGamepad1.updateGamepadState(gamepad1)

        waitForStart()

        while (isStarted && !isStopRequested) {
            dashboardTelemetry.addData("Current", motor.currentPosition)
            dashboardTelemetry.addData("Target", linearSlide.motor.targetPosition)
            dashboardTelemetry.update()

            if (gamepad1.a && !pastGamepad1.a) {
                    linearSlide.extend(1.0)
            } else if (gamepad1.b && !pastGamepad1.b) {
                linearSlide.extend(-1.0)
            }

            pastGamepad1.updateGamepadState(gamepad1)
        }
    }
}