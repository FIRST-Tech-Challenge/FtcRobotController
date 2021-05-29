package org.firstinspires.ftc.teamcode.buttons.dpad

import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.autonomous.DriveFunctions

class Up(robot: Robot) {
    init {
        DriveFunctions(robot).testEncoderDrive()
    }
}