package org.firstinspires.ftc.teamcode.buttons

import kotlinx.coroutines.Deferred
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.async
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.Robot

// Add functions to their appropriate package for readability then restate them here

class AllMappings(robot: Robot) {

    val robot = robot

    lateinit var looper: Deferred<Unit>;

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
    }

    fun dpadUp() {
        org.firstinspires.ftc.teamcode.buttons.dpad.Up(this.robot)
    }
}