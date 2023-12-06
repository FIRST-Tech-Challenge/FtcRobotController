package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.BotShared

@TeleOp(name = "Claw Test", group = "Test")
class ClawTest : OpMode() {

    private lateinit var shared: BotShared

    override fun init() {
        shared = BotShared(this)
    }

    override fun loop() {
        val claw = shared.claw!!
        val clawLeft = shared.servoClawLeft!!
        val clawRight = shared.servoClawRight!!
        clawLeft.direction = Servo.Direction.REVERSE
        clawRight.direction = Servo.Direction.FORWARD

        clawLeft.position += -gamepad2.left_stick_y * 0.01
        clawRight.position += -gamepad2.right_stick_y * 0.01

        telemetry.addLine("Claw Positions: L=${shared.servoClawLeft?.position} R=${shared.servoClawRight?.position}")
        telemetry.addLine("Claw Module: ${claw.state}")
        telemetry.addLine("Left Stick -Y: " + -gamepad2.left_stick_y)
        telemetry.addLine("Right Stick -Y: " + -gamepad2.right_stick_y)
        telemetry.update()

        shared.update()
    }
}