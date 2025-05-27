package org.firstinspires.ftc.teamcode.scripts

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.internals.templates.Script

class ServoScript( //script for controlling a servo
    private val servoId: Int,
    private val input:()-> Double = { (gamepad1.right_trigger-gamepad1.left_trigger).toDouble() }
): Script() {

    private val servo = hardwareMap.get(Servo::class.java, "servo$servoId")

    override fun init() {
    }

    override fun run() {
        while (scriptIsActive()) {
            servo.position = input()
        }
    }

    override fun onStop() {
    }
}