package org.firstinspires.ftc.teamcode.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.SwerveModule
import org.firstinspires.ftc.teamcode.Wrist
import org.firstinspires.ftc.teamcode.WristCont
import org.firstinspires.ftc.teamcode.supers.Robot

@TeleOp(name="SwerveModuleTest")
class SwerveModuleTest: LinearOpMode() {
    override fun runOpMode() {
        val topMotor: DcMotor = hardwareMap.get(DcMotor::class.java, "one")
        val bottomMotor: DcMotor = hardwareMap.get(DcMotor::class.java, "two")
        val module = SwerveModule(topMotor, bottomMotor, 537.7, 40.0/120.0)

        val dashboard: FtcDashboard? = FtcDashboard.getInstance()
        val dashboardTelemetry = dashboard!!.telemetry

        waitForStart()

        while (isStarted && !isStopRequested) {
            module.topMotor.power = gamepad1.left_stick_y.toDouble()
            module.bottomMotor.power = gamepad1.right_stick_y.toDouble()

            dashboardTelemetry.addData("Angle", module.getAngle())
            //dashboardTelemetry.addData("Twist", wrist.getTwist())
            //dashboardTelemetry.addData("L", wrist.lServoTarget)
            //dashboardTelemetry.addData("R", wrist.rServoTarget)
            dashboardTelemetry.update()
        }
    }

}