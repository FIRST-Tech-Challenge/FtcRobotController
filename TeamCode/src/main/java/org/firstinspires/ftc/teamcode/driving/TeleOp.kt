@file:Suppress("CanBeVal", "SpellCheckingInspection")

package org.firstinspires.ftc.teamcode.driving

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Hardware
import kotlin.math.abs

@Suppress("UNUSED_VARIABLE")
@TeleOp(name = "main teleop", group = "Prod")
class TeleOp: LinearOpMode() {
    override fun runOpMode() {
        var hardware = Hardware(hardwareMap)

        val frontLeftMotor = Hardware.motorTL
        val backLeftMotor = Hardware.motorBL
        val backRightMotor = Hardware.motorBR
        val frontRightMotor = Hardware.motorTR
        waitForStart()
        while (opModeIsActive()) {
            val y: Double = (-gamepad1.left_stick_y).toDouble() // Remember, Y stick value is reversed
            val x: Double = (gamepad1.left_stick_x).toDouble() // Counteract imperfect strafing
            val rx: Double = (gamepad1.right_stick_x).toDouble()

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            var denominator: Double = (abs(y) + abs(x) + abs(rx)).coerceAtMost(1.0)
            var frontLeftPower: Double = (y+x+rx) / denominator
            var backLeftPower: Double = (y-x+rx) / denominator
            var frontRightPower: Double = (y-x-rx) / denominator
            var backRightPower: Double = (y+x-rx) / denominator

            frontLeftMotor.power = frontLeftPower
            backLeftMotor.power = backLeftPower
            frontRightMotor.power = frontRightPower
            backRightMotor.power = backRightPower
        }
    }

}