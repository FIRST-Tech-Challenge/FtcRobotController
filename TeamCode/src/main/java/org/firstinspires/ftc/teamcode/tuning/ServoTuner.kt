package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.ServoHand
import org.firstinspires.ftc.teamcode.util.GamepadState

@TeleOp(name="ServoTuner")
class ServoTuner : LinearOpMode() {
    override fun runOpMode() {
        val servo: Servo = hardwareMap.get(Servo::class.java, "s_one")

        val dashboard: FtcDashboard? = FtcDashboard.getInstance()
        val dashboardTelemetry = dashboard!!.telemetry

        waitForStart()

        while (isStarted && !isStopRequested) {
            servo.position = gamepad1.left_stick_y.toDouble()
            dashboardTelemetry.addData("Position", servo.position)
            dashboardTelemetry.update()
        }
    }
}