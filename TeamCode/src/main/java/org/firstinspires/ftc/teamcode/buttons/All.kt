package org.firstinspires.ftc.teamcode.buttons

import com.qualcomm.robotcore.hardware.Gamepad
import kotlinx.coroutines.Deferred
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.async
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.buttons.dpad.Left
import org.firstinspires.ftc.teamcode.buttons.dpad.Right
import org.firstinspires.ftc.teamcode.buttons.dpad.Up
import org.firstinspires.ftc.teamcode.driver.EncoderDrive
import org.firstinspires.ftc.teamcode.imgproc.ImgProc
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.DriveTrain

// Add functions to their appropriate package for readability then restate them here

@Deprecated("Deprecated in favor of ButtonMGR in Java")
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
        var gamepad = robot.getRobotPart(Gamepad::class.java) as Gamepad
        if (gamepad.dpad_up) {
            this.dpadUp()
        }

        if (gamepad.dpad_right) {
            this.dpadRight()
        }

        if (gamepad.dpad_left) {
            this.dpadLeft()
        }

        if (gamepad.left_bumper) {
            this.leftBumper()
        }
    }

    private fun dpadUp() {
        this.encoder = Up(this.robot).encoder
    }

    private fun dpadRight() {
        this.image = Right(this.telemetry, ImgProc()).img
    }

    private fun dpadLeft() {
        Left(this.image)
    }

    private fun leftBumper() {
        org.firstinspires.ftc.teamcode.buttons.bumpers.Left(this.encoder)
    }
}