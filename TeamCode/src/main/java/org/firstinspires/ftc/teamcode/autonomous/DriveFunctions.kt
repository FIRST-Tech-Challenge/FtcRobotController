package org.firstinspires.ftc.teamcode.autonomous

import org.firstinspires.ftc.teamcode.driver.EncoderDrive
import org.firstinspires.ftc.teamcode.robot.Robot

class DriveFunctions(robot: Robot) {

    val encoder = EncoderDrive(robot)

    fun testEncoderDrive() = this.encoder.encoderDrive(50.0, 30.0, 30.0, 0.0, 10.0)
}