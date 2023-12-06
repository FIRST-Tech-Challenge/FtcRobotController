package org.firstinspires.ftc.teamcode.botmodule

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo

class DroneLauncher(opMode: OpMode, private var launchServo: Servo) : BotModule(opMode) {

    init {
        launchServo.position = 0.0
    }

    fun launch() {
        launchServo.position = 1.0
    }

//    public fun launchAsync(launchWithoutArming: Boolean?): Boolean { }
}