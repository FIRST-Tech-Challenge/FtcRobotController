package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Wrist
import org.firstinspires.ftc.teamcode.WristCont
import org.firstinspires.ftc.teamcode.supers.Robot

@TeleOp(name="WristTest")
class WristTest: LinearOpMode() {
    override fun runOpMode() {
        val lServo: CRServo = hardwareMap.get(CRServo::class.java, "lServo")
        val rServo: CRServo = hardwareMap.get(CRServo::class.java, "rServo")
        val wrist = WristCont(lServo, rServo)

        val dashboard: FtcDashboard? = FtcDashboard.getInstance()
        val dashboardTelemetry = dashboard!!.telemetry

        waitForStart()

        while (isStarted && !isStopRequested) {
            wrist.twist(gamepad1.left_stick_y.toDouble() / 1)
            wrist.turn(gamepad1.left_stick_x.toDouble() / 1)

            //dashboardTelemetry.addData("Turn", wrist.getTurn())
            //dashboardTelemetry.addData("Twist", wrist.getTwist())
            //dashboardTelemetry.addData("L", wrist.lServoTarget)
            //dashboardTelemetry.addData("R", wrist.rServoTarget)
            dashboardTelemetry.update()
        }
    }

}