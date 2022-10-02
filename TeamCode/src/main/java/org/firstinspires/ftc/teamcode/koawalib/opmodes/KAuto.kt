package org.firstinspires.ftc.teamcode.koawalib.opmodes

import com.asiankoala.koawalib.command.KOpMode
import org.firstinspires.ftc.teamcode.koawalib.Robot

class KAuto : KOpMode() {
    private val robot by lazy { Robot() }

    override fun mInit() {
        robot.webcam.device.startStreaming()
    }

    override fun mInitLoop() {
        robot.webcam.periodic()
    }

    override fun mStart() {
        robot.webcam.device.stopStreaming()
    }
}