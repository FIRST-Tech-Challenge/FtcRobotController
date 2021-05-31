package org.firstinspires.ftc.teamcode.buttons

import kotlinx.coroutines.Deferred
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.async
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.buttons.dpad.*
import org.firstinspires.ftc.teamcode.driver.EncoderDrive
import org.firstinspires.ftc.teamcode.imgproc.ImgProc

// Add functions to their appropriate package for readability then restate them here

class AllMappings(robot: Robot, telemetry: Telemetry) {

    val robot = robot

    val telemetry = telemetry

    lateinit var looper: Deferred<Unit>

    lateinit var image: ImgProc

    lateinit var encoder: EncoderDrive

    fun start() {
        this.looper = GlobalScope.async(Dispatchers.Default) {
            loop()
        }
    }

    fun stop() {
        this.looper.cancel()
    }

    private suspend fun loop() {
        if (this.robot.gamepad1.dpad_up) {
            this.dpadUp()
        }

        if (this.robot.gamepad1.dpad_right) {
            this.dpadRight()
        }

        if (this.robot.gamepad1.dpad_left) {
            this.dpadRight()
        }

        if (this.robot.gamepad1.left_bumper) {
            this.leftBumper()
        }
    }

    fun dpadUp() {
        this.encoder = Up(this.robot).encoder
    }

    fun dpadRight() {
        this.image = Right(this.telemetry).img
    }

    fun dpadLeft() {
        Left(this.image)
    }

    fun leftBumper() {
        org.firstinspires.ftc.teamcode.buttons.bumpers.Left(this.encoder)
    }
}