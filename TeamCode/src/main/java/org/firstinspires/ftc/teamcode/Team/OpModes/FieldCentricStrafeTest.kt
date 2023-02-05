package org.firstinspires.ftc.teamcode.Team.OpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.teamcode.Team.ComplexRobots.Robot
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@TeleOp
class FieldCentricStrafeTest : LinearOpMode() {
    var robot = Robot()
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot.init(hardwareMap)

        val imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            val y = -gamepad1.left_stick_y.toDouble()
            val x = gamepad1.left_stick_x * 1.1
            val rx = gamepad1.right_stick_x.toDouble()
            val botHeading = -imu.angularOrientation.firstAngle.toDouble()
            val rotX = x * cos(botHeading) - y * sin(botHeading)
            val rotY = x * sin(botHeading) + y * cos(botHeading)
            val denominator = (abs(y) + abs(x) + abs(rx)).coerceAtLeast(1.0)

            val lfpower = (rotY + rotX + rx) / denominator
            val lbpower = (rotY - rotX + rx) / denominator
            val rfpower = (rotY - rotX - rx) / denominator
            val rbpower = (rotY + rotX - rx) / denominator

            robot.setPower(mapOf(
                "LFMotor" to lfpower,
                "LBMotor" to lbpower,
                "RFMotor" to rfpower,
                "RBMotor" to rbpower,
            ))
        }
    }
}