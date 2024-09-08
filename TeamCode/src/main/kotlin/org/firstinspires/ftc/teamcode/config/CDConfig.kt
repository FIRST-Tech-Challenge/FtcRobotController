package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.config.components.DriveMotors
import org.firstinspires.ftc.teamcode.config.components.WheelLocalizers

data class CDConfig(
    val driveMotors: DriveMotors = DriveMotors(),
    val wheelLocalizers: WheelLocalizers = WheelLocalizers(),
    val direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    val debugMode: Boolean = false
)