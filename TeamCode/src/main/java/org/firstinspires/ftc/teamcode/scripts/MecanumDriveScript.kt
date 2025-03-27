package org.firstinspires.ftc.teamcode.scripts


import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.internals.templates.Script

class MecanumDriveScript(
    private val bevelGears: Boolean = true,
    private val powerScale: Double = 1.0,
    private val rotScale: Double = 1.0,
    private val forward:()-> Double = { gamepad1.left_stick_y.toDouble() },
    private val lateral:()-> Double = { gamepad1.left_stick_x.toDouble() },
    private val rotation:()-> Double = { gamepad1.right_stick_x.toDouble() },
    private val flId:()-> Int = { 0 },
    private val blId:()-> Int = { 1 },
    private val frId:()-> Int = { 2 },
    private val brId:()-> Int = { 3 }

): Script() {
    val fl = hardwareMap.dcMotor.get("motor$flId")
    val bl = hardwareMap.dcMotor.get("motor1$blId")
    val fr = hardwareMap.dcMotor.get("motor2$frId")
    val br = hardwareMap.dcMotor.get("motor3$brId")

    override fun init() {}

    override fun run() {
        while(scriptIsActive()){
            val y = forward()
            val x = lateral()
            val r = rotation() * rotScale

            val denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1.0)
            var frontLeftPower = (y + x + r) / denominator
            var backLeftPower = (y - x + r) / denominator
            var frontRightPower = (y - x - r) / denominator
            var backRightPower = (y + x - r) / denominator

            if (bevelGears) {
                frontLeftPower *= -1
                backLeftPower *= -1
                frontRightPower *= -1
                backRightPower *= -1
            }

            frontLeftPower *= powerScale
            backLeftPower *= powerScale
            frontRightPower *= -powerScale
            backRightPower *= -powerScale


        }
    }

    override fun onStop() {}


}