package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.supers.Robot

@TeleOp(name = "Test Swerve Forward")
@Config
class TestSwerveForward : LinearOpMode() {
    var power: Double = 0.5

    override fun runOpMode() {
        val r = Robot(this)

        waitForStart()

        while (opModeIsActive()) {
            power = -gamepad2.right_stick_y.toDouble() / 3.5

            r.dashboardTelemetry.addData("A", power)
            r.dashboardTelemetry.addData("LT", r.sl.top.currentPosition)
            r.dashboardTelemetry.addData("LB", r.sl.bottom.currentPosition)
            r.dashboardTelemetry.addData("RT", r.sr.top.currentPosition)
            r.dashboardTelemetry.addData("RB", r.sr.bottom.currentPosition)
            r.dashboardTelemetry.update()

            r.sl.top.power = power
            r.sl.bottom.power = power

            r.sr.top.power = power
            r.sr.bottom.power = power
        }

    }
}