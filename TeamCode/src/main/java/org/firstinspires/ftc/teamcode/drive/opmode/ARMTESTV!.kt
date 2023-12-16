package org.firstinspires.ftc.teamcode.drive.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx

@TeleOp(name="Arm Test")
class armTestV1 : LinearOpMode() {
    private lateinit var motor1: DcMotorEx
    private lateinit var motor2: DcMotorEx
    override fun runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx::class.java,"motor6")
        motor2 = hardwareMap.get(DcMotorEx::class.java, "motor7")

        motor2.direction.inverted()
        waitForStart()
        while (opModeIsActive()) {
            motor1.setPower(gamepad1.left_stick_y.toDouble())
            motor2.setPower(gamepad1.left_stick_y.toDouble())
        }

    }

}