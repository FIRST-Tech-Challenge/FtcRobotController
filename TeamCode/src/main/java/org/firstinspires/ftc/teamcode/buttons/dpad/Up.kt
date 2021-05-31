package org.firstinspires.ftc.teamcode.buttons.dpad

import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.autonomous.DriveFunctions
import org.firstinspires.ftc.teamcode.driver.EncoderDrive

class Up(robot: Robot) {

    lateinit var encoder: EncoderDrive

    init {
        val driver = DriveFunctions(robot)
        driver.testEncoderDrive()
        this.encoder = driver.encoder
    }
}