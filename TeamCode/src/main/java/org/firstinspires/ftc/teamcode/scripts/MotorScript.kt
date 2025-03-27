package org.firstinspires.ftc.teamcode.scripts

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.internals.templates.Script

class MotorScript(
    private val motorId: Int,
    private val scale: Double = 1.0,
    private val input:()-> Double = { (gamepad1.right_trigger-gamepad1.left_trigger).toDouble() }
): Script() {

    private val motor = hardwareMap.get(DcMotor::class.java, "motor$motorId")

    override fun init(){}


    override fun run() {
        while (scriptIsActive()) {
            motor.power = input()*scale
        }
    }

    override fun onStop() {
    }
}