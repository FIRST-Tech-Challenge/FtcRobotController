package org.firstinspires.ftc.teamcode.koawalib

import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive

class Robot(startPose: Pose) {
    val hardware = Hardware(startPose)

    val drive = KMecanumOdoDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr,
        hardware.odometry,
        true
    )

    val arm = Arm(hardware.arm)

    init {
        arm.motor.setPositionTarget(Arm.homePosition)
    }
}