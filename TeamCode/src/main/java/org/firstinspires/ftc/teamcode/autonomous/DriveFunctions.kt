package org.firstinspires.ftc.teamcode.autonomous

import org.firstinspires.ftc.teamcode.driver.EncoderDrive
import org.firstinspires.ftc.teamcode.robot.Robot

class DriveFunctions(robot: Robot) {

    private val encoder = EncoderDrive(robot)

    fun testEncoderDrive() {
        encoder.encoderDrive(50.0, 30.0, 30.0)
    }

}