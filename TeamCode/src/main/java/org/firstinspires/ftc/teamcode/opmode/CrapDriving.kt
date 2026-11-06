package org.firstinspires.ftc.teamcode.opmode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Configurable
@TeleOp
class CrapDriving : OpMode() {
    val frontLeft by lazy {
        hardwareMap["frontLeft"] as DcMotorEx
    }
    val backLeft by lazy {
        hardwareMap["backLeft"] as DcMotorEx
    }
    val frontRight by lazy {
        hardwareMap["frontRight"] as DcMotorEx
    }
    val backRight by lazy {
        hardwareMap["backRight"] as DcMotorEx
    }
    override fun init() {
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        frontRight
        backRight
    }

    override fun loop() {
        for (motor in listOf(
            frontRight,
            frontLeft,
            backRight,
            backLeft
        )) {
            motor.power = gamepad1.left_stick_y.toDouble()
        }
    }
}