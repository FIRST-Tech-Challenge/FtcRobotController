package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.Telemetry

@TeleOp(name="Basic Kotlin TeleOp") // Compatible with Java annotations
class BasicTeleOp : OpMode() {
    // the lateinit modifier allows you to wait to initialize a variable in a neat fashion
    private lateinit var leftDrive: DcMotor
    private lateinit var rightDrive: DcMotor

    override fun init() {
        leftDrive = hardwareMap.dcMotor.get("left_drive")
        rightDrive = hardwareMap.dcMotor.get("right_drive")
    }

    override fun loop() {
        val leftPower = gamepad1.left_stick_y
        val rightPower = gamepad1.right_stick_y

        // Instead of using getters/setters, just let kotlin do it for you!
        leftDrive.power = leftPower.toDouble()
        rightDrive.power = rightPower.toDouble()

        // This part is basically synonymous with java
        telemetry.addData("Left Power", leftPower)
        telemetry.addData("Right Power", rightPower)
        telemetry.update()
    }

}