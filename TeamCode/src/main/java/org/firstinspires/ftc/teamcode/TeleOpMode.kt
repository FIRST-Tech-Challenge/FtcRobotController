package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "Teleop main", group = "")
class TeleOpMode : LinearOpMode() {
    lateinit var motorLeft : DcMotor;
    lateinit var motorRight : DcMotor;
    override fun runOpMode() {
        motorRight = hardwareMap.get(DcMotor::class.java, "motor_right")
        motorLeft = hardwareMap.get(DcMotor::class.java, "motor_left")

        motorRight.direction = DcMotorSimple.Direction.FORWARD
        motorLeft.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        while (opModeIsActive()) {
            val gX = gamepad1.right_stick_x.toDouble()
            val gY = gamepad1.right_stick_y.toDouble()

            motorRight.power = gY - gX
            motorLeft.power = gY + gX

            telemetry.addData("Stick x", gX)
            telemetry.addData("Stick y", gY)
            telemetry.update()
        }
    }

}